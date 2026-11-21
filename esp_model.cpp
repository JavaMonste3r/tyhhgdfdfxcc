#include "esp_model.h"
#include "esp_pulse.h"

#include <windows.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace omega::esp::model
{
    namespace
    {
        Il2CppResolver *g_resolver = nullptr;
        void (*g_log)(const char *) = nullptr;
        char g_status[192] = "model: off";
        // Set by Tick: the flag file is present and no sentinel disables it.
        // esp.cpp uses it to keep the managed walk off the render thread
        // entirely, including the minutes a fresh locate scan can take.
        bool g_requested = false;

        // --- BCL layout -----------------------------------------------------
        // These belong to the runtime, not to the game: il2cpp's
        // System.Collections.Generic.Dictionary keeps _buckets at 0x10,
        // _entries at 0x18, _count at 0x20, and a System.Array keeps its
        // element data at 0x20 (length 0x18). Each entry is 0x18 bytes:
        // hashCode 0x00, next 0x04, key 0x08, value 0x10.
        constexpr size_t kDictEntries = 0x18;
        constexpr size_t kDictCount = 0x20;
        constexpr size_t kArrayData = 0x20;
        constexpr size_t kEntryStride = 0x18;
        constexpr size_t kEntryKey = 0x08;
        constexpr size_t kEntryValue = 0x10;

        // BasePrimitiveData<T> / BasePrimitiveModel<T>: the payload sits after
        // the serializer pointer, at +0x18 on every primitive holder.
        constexpr size_t kPrimitiveValue = 0x18;

        // A corrupt length must never drive a read: the live lobby tops out far
        // below this, and the game's own dictionary is bounded by the same idea.
        constexpr int kMaxCharacters = 512;
        // Locate scan budget per slice. Measured on the live game: 8 MB of qword
        // compare costs ~15 ms inside Present, i.e. a tenth of the frame, every
        // frame, for as long as no graph was located. 1 MB per 50 ms slice is
        // the same search at a cost the frame does not notice.
        constexpr size_t kScanBytesPerTick = 4u * 1024u * 1024u;
        // Skip regions the allocator hands out for stacks and shared memory:
        // the GC heap is committed private memory, and the string is short.
        constexpr unsigned long long kSnapshotTtlMs = 33;

        struct Layout
        {
            bool resolved = false;
            size_t battleEntities = 0;   // BattleModel -> IBattleEntitiesModel
            size_t worldEntities = 0;    // BattleEntitiesModel -> WorldEntitiesModel
            size_t player = 0;           // BattleEntitiesModel -> PlayerEntityModel
            size_t characters = 0;       // WorldEntitiesModel -> Dictionary<int, BattleCharacterModel>
            size_t serverModel = 0;      // PlayerEntityModel -> BattleCharacterModel
            size_t transform = 0;        // BattleCharacterModel -> ITransformModel
            size_t data = 0;             // BattleCharacterModel -> BattleCharacterData
            size_t id = 0;               // BattleCharacterModel -> int Id
            size_t commonData = 0;
            size_t privateData = 0;
            size_t publicData = 0;
            size_t role = 0;             // BattleCharacterCommonData -> IPrimitiveData<PlayerRoleDescription>
            size_t roleId = 0;           // PlayerRoleDescription -> Identified.<Id> (string)
            size_t hasArmor = 0;         // BattleCharacterPublicData -> IPrimitiveData<bool>
            size_t health = 0;           // BattleCharacterPrivateData -> IPrimitiveData<int>
            size_t slotModel = 0;        // BattleEntitiesModel -> ClientCharacterSlotsModel
            size_t slotByEntity = 0;     // ClientCharacterSlotsModel -> Dictionary<int, BattleCharacterSlotModel>
            size_t slotData = 0;         // BattleCharacterSlotModel -> BattleCharacterSlotData
            size_t slotNickname = 0;     // BattleCharacterSlotData -> IPrimitiveData<string>
            size_t transformX = 0;
            size_t transformY = 0;
            size_t transformZ = 0;
        };

        Layout g_layout;
        void *g_battleModelClass = nullptr;

        // Locate scan cursor: the graph object is a plain managed reference with
        // no static holder, so it is found once by looking for an object whose
        // klass field is BattleModel. The scan walks committed writable regions
        // a slice at a time and resumes on the next tick.
        bool g_scanning = false;
        unsigned long long g_cursor = 0;
        // A finished sweep waits this long before starting over: the graph does
        // not exist until a match begins, and a sweep is a few GB of reads.
        constexpr unsigned long long kScanRetryMs = 5000;
        unsigned long long g_retryAtMs = 0;

        std::vector<unsigned char> g_region;
        void *g_entities = nullptr;
        void *g_world = nullptr;
        void *g_local = nullptr;
        // Every managed instance starts with its class pointer. The locate scan
        // matches a qword equal to the BattleModel class anywhere in private
        // memory, so a stack dump or a plain float table can match by accident;
        // checking the header of what the field points at is what tells a real
        // object from garbage. Evidence of the accident: a run logged
        // `model: graph accepted, entities=0x7FFCAF0DD9D0 world=0x7FFCAF05C8A0`
        // (both in mapped image range, one character in the dictionary, zero in
        // the snapshot a millisecond later).
        void *g_entitiesKlass = nullptr;
        void *g_worldKlass = nullptr;
        std::vector<Character> g_characters;
        unsigned long long g_version = 0;
        unsigned long long g_lastSnapshot = 0;
        // Statistics for the diagnostics tab.
        int g_locateAttempts = 0;

        // Bisect switch: an empty file named OmegaCore_nomodel in %TEMP% turns
        // the whole module off without a rebuild, so one inject can tell whether
        // a crash lives in this path or not.
        bool DisabledBySentinel()
        {
            static int cached = -1;
            if (cached >= 0)
            {
                return cached != 0;
            }
            cached = 0;
            wchar_t path[MAX_PATH]{};
            if (GetTempPathW(MAX_PATH, path) != 0)
            {
                lstrcatW(path, L"OmegaCore_nomodel");
                if (GetFileAttributesW(path) != INVALID_FILE_ATTRIBUTES)
                {
                    cached = 1;
                }
            }
            return cached != 0;
        }

        // Opt-in switch: the graph locate scan costs frame time and its crash
        // safety is unproven (see GOAL "СЕЙЧАС ДЕЛАЮ"), so the module is dark
        // unless an empty file %TEMP%\OmegaCore_model exists. Turn it on with a
        // new inject when the nickname/role chain is being worked on.
        bool EnabledByFlag()
        {
            static int cached = -1;
            if (cached >= 0)
            {
                return cached != 0;
            }
            cached = 0;
            wchar_t path[MAX_PATH]{};
            if (GetTempPathW(MAX_PATH, path) != 0)
            {
                lstrcatW(path, L"OmegaCore_model");
                if (GetFileAttributesW(path) != INVALID_FILE_ATTRIBUTES)
                {
                    cached = 1;
                }
            }
            return cached != 0;
        }

        void Log(const char *format, ...)
        {
            if (g_log == nullptr)
            {
                return;
            }
            char text[192];
            va_list args;
            va_start(args, format);
            std::vsnprintf(text, sizeof(text), format, args);
            va_end(args);
            g_log(text);
        }

        bool ReadMem(const void *address, void *out, size_t size)
        {
            if (address == nullptr || out == nullptr || size == 0)
            {
                return false;
            }
            SIZE_T got = 0;
            // In-process ReadProcessMemory is the cheap way to turn a stale
            // managed pointer into FALSE instead of an access violation on the
            // render thread.
            pulse::ReadScope reading(address);
            if (!ReadProcessMemory(GetCurrentProcess(), address, out, size, &got))
            {
                return false;
            }
            return got == size;
        }

        template <typename T>
        bool ReadField(const void *base, size_t offset, T &out)
        {
            if (base == nullptr)
            {
                return false;
            }
            return ReadMem(static_cast<const char *>(base) + offset, &out, sizeof(T));
        }

        bool Plausible(const void *pointer)
        {
            const auto value = reinterpret_cast<uintptr_t>(pointer);
            return value >= 0x10000u && (value & 7u) == 0u && value < 0x00007FFFFFFEFFFFull;
        }

        // True when `object` is an instance of `klass` (il2cpp stores the class
        // pointer in the first qword of every managed object).
        bool InstanceOf(const void *object, void *klass)
        {
            if (klass == nullptr || !Plausible(object))
            {
                return false;
            }
            void *header = nullptr;
            if (!ReadField(object, 0, header))
            {
                return false;
            }
            return header == klass;
        }

        // Il2CppResolver::FieldOffset answers (size_t)-1 when the name is not on
        // the class, so "missing" has an explicit spelling here.
        constexpr size_t kMissing = static_cast<size_t>(-1);

        size_t Offset(void *klass, const char *name)
        {
            if (klass == nullptr || name == nullptr)
            {
                return kMissing;
            }
            return g_resolver->FieldOffset(klass, name);
        }

        bool Has(size_t offset)
        {
            return offset != kMissing && offset != 0;
        }


        // Every field is looked up by name; a missing name only disables the
        // value that depends on it, never the whole path.
        bool ResolveLayout()
        {
            if (g_layout.resolved)
            {
                return true;
            }
            if (g_resolver == nullptr || !g_resolver->Ready())
            {
                return false;
            }

            void *battle = g_resolver->FindClassAnyImage("Game.Battle", "BattleModel");
            void *entities = g_resolver->FindClassAnyImage("Game.Battle", "BattleEntitiesModel");
            void *world = g_resolver->FindClassAnyImage("Models.Models.WorldEntities", "WorldEntitiesModel");
            void *player = g_resolver->FindClassAnyImage("Game.Battle.EntityModel.Player", "PlayerEntityModel");
            void *character = g_resolver->FindClassAnyImage("Models.Models.BattleCharacter", "BattleCharacterModel");
            void *data = g_resolver->FindClassAnyImage("Models.Data", "BattleCharacterData");
            void *common = g_resolver->FindClassAnyImage("Models.Data", "BattleCharacterCommonData");
            void *privateData = g_resolver->FindClassAnyImage("Models.Data", "BattleCharacterPrivateData");
            void *publicData = g_resolver->FindClassAnyImage("Models.Data", "BattleCharacterPublicData");
            void *transform = g_resolver->FindClassAnyImage("Models.Models.TransformModel", "CharacterTransformModel");

            if (battle == nullptr || entities == nullptr || world == nullptr || character == nullptr)
            {
                std::snprintf(g_status, sizeof(g_status),
                              "model: class missing (battle=%d entities=%d world=%d char=%d)",
                              battle != nullptr, entities != nullptr, world != nullptr,
                              character != nullptr);
                return false;
            }

            g_battleModelClass = battle;
            g_entitiesKlass = entities;
            g_worldKlass = world;

            g_layout.battleEntities = Offset(battle, "<BattleEntitiesModel>k__BackingField");
            g_layout.worldEntities = Offset(entities, "<WorldEntitiesModel>k__BackingField");
            if (!Has(g_layout.worldEntities))
            {
                // The world list is a plain readonly field on this build.
                g_layout.worldEntities = Offset(entities, "_worldEntitiesModel");
            }

            g_layout.player = Offset(entities, "<Player>k__BackingField");
            g_layout.characters = Offset(world, "_battleCharacters");
            g_layout.serverModel = Offset(player, "<ServerModel>k__BackingField");
            g_layout.transform = Offset(character, "<Transform>k__BackingField");
            g_layout.data = Offset(character, "<Data>k__BackingField");
            g_layout.id = Offset(character, "<Id>k__BackingField");
            g_layout.commonData = Offset(data, "<CommonData>k__BackingField");
            g_layout.privateData = Offset(data, "<PrivateData>k__BackingField");
            g_layout.publicData = Offset(data, "<PublicData>k__BackingField");
            g_layout.role = Offset(common, "<Role>k__BackingField");
            g_layout.roleId = Offset(
                g_resolver->FindClassAnyImage("Models.References.Battle", "PlayerRoleDescription"),
                "<Id>k__BackingField");

            g_layout.hasArmor = Offset(publicData, "<HasArmor>k__BackingField");
            g_layout.health = Offset(privateData, "<Health>k__BackingField");

            // Nicknames live on the slot list, not on the character: the slot
            // dictionary is keyed by the same entity id the character carries,
            // so one hop from BattleEntitiesModel reaches every name.
            void *slotModelClass =
                g_resolver->FindClassAnyImage("ClientCore.CharacterSlotsModel", "ClientCharacterSlotsModel");
            void *slotEntryClass =
                g_resolver->FindClassAnyImage("Models.Models.BattleCharacterSlots", "BattleCharacterSlotModel");
            void *slotDataClass = g_resolver->FindClassAnyImage("Models.Data", "BattleCharacterSlotData");
            g_layout.slotModel = Offset(entities, "_characterSlotModel");
            g_layout.slotByEntity = Offset(slotModelClass, "_slotByEntityId");
            g_layout.slotData = Offset(slotEntryClass, "_data");
            g_layout.slotNickname = Offset(slotDataClass, "<Nickname>k__BackingField");
            g_layout.transformX = Offset(transform, "<X>k__BackingField");
            g_layout.transformY = Offset(transform, "<Y>k__BackingField");
            g_layout.transformZ = Offset(transform, "<Z>k__BackingField");

            if (!Has(g_layout.battleEntities) || !Has(g_layout.worldEntities) ||
                !Has(g_layout.characters) || !Has(g_layout.transform) ||
                !Has(g_layout.transformX) || !Has(g_layout.transformY) ||
                !Has(g_layout.transformZ))
            {
                std::snprintf(g_status, sizeof(g_status),
                              "model: field missing (entities=%zd world=%zd chars=%zd transform=%zd)",
                              static_cast<ptrdiff_t>(g_layout.battleEntities),
                              static_cast<ptrdiff_t>(g_layout.worldEntities),
                              static_cast<ptrdiff_t>(g_layout.characters),
                              static_cast<ptrdiff_t>(g_layout.transform));
                return false;
            }

            g_layout.resolved = true;
            if (g_log != nullptr)
            {
                char text[192];
                std::snprintf(text, sizeof(text),
                              "model: layout ok (entities=0x%zX world=0x%zX chars=0x%zX player=0x%zX)",
                              g_layout.battleEntities, g_layout.worldEntities,
                              g_layout.characters, g_layout.player);
                g_log(text);
            }
            return true;
        }

        // Reads enough of the graph to prove `candidate` is the live BattleModel.
        // A graph whose character dictionary is empty is a stale or prefab model:
        // the live one always carries the characters currently in the world.
        bool Validate(void *candidate, int &countOut)
        {
            countOut = 0;
            void *entities = nullptr;
            if (!ReadField(candidate, g_layout.battleEntities, entities) ||
                !InstanceOf(entities, g_entitiesKlass))
            {
                return false;
            }
            void *world = nullptr;
            if (!ReadField(entities, g_layout.worldEntities, world) ||
                !InstanceOf(world, g_worldKlass))
            {
                return false;
            }
            void *dictionary = nullptr;
            if (!ReadField(world, g_layout.characters, dictionary) || !Plausible(dictionary))
            {
                return false;
            }

            int count = 0;
            if (!ReadField(dictionary, kDictCount, count) || count < 0 || count > kMaxCharacters)
            {
                return false;
            }

            g_entities = entities;
            g_world = world;
            countOut = count;
            return true;
        }

        // One slice of the locate scan. Returns true when a valid graph was
        // assigned to g_entities/g_world.
        bool ScanSlice()
        {
            if (g_battleModelClass == nullptr)
            {
                return false;
            }
            // The scan runs on the render thread inside Present, so it is a
            // direct subtraction from the frame. At 8 MiB per frame it cost 15 ms
            // of EVERY frame while no graph was located — that is the "esp lags"
            // report: the overlay was stealing a tenth of the game's frame.
            // A slice every 50 ms at 1 MiB is 4 % of one frame, slow to find and
            // invisible when it does not find.
            static unsigned long long lastSliceTick = 0;
            const unsigned long long sliceNow = GetTickCount64();
            if (sliceNow - lastSliceTick < 50)
            {
                return false;
            }
            lastSliceTick = sliceNow;
            if (!g_scanning && sliceNow < g_retryAtMs)
            {
                return false;
            }
            SYSTEM_INFO info;
            GetSystemInfo(&info);
            const auto minimum =
                reinterpret_cast<uintptr_t>(info.lpMinimumApplicationAddress);
            const auto maximum =
                reinterpret_cast<uintptr_t>(info.lpMaximumApplicationAddress);

            if (!g_scanning)
            {
                g_cursor = minimum;
                g_scanning = true;
                ++g_locateAttempts;
            }

            size_t budget = kScanBytesPerTick;
            while (g_cursor < maximum && budget > 0)
            {
                MEMORY_BASIC_INFORMATION region{};
                if (VirtualQuery(reinterpret_cast<LPCVOID>(g_cursor), &region, sizeof(region)) !=
                    sizeof(region))
                {
                    g_scanning = false;
                    return false;
                }

                const auto base = reinterpret_cast<uintptr_t>(region.BaseAddress);
                const auto size = region.RegionSize;
                const bool readable =
                    region.State == MEM_COMMIT &&
                    (region.Protect & (PAGE_READONLY | PAGE_READWRITE | PAGE_WRITECOPY |
                                       PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE |
                                       PAGE_EXECUTE_WRITECOPY)) != 0 &&
                    (region.Protect & PAGE_GUARD) == 0 && region.Type == MEM_PRIVATE;

                size_t consumed = size;
                if (readable && size >= sizeof(void *))
                {
                    // Never read past the per-call budget, and never skip the
                    // tail of a region bigger than the budget: the cursor moves
                    // by what was actually read.
                    const size_t slice = std::min<size_t>(size, budget);
                    budget -= slice;
                    consumed = slice;
                    g_region.resize(slice);

                    if (ReadMem(reinterpret_cast<const void *>(base), g_region.data(), slice))
                    {
                        const auto want = reinterpret_cast<uintptr_t>(g_battleModelClass);
                        const unsigned char *bytes = g_region.data();
                        // memchr on the low byte first: a GC heap is mostly
                        // floats and small ints, so the first byte hit cuts the
                        // qword compares by an order of magnitude.
                        size_t at = 0;
                        while (at + sizeof(void *) <= slice)
                        {
                            const void *hit = std::memchr(bytes + at, static_cast<int>(want & 0xFF), slice - at - (sizeof(void *) - 1));
                            if (hit == nullptr)
                            {
                                break;
                            }
                            const size_t index = static_cast<size_t>(
                                static_cast<const unsigned char *>(hit) - bytes);
                            at = index + 1;
                            if ((index & 7u) != 0)
                            {
                                continue;
                            }
                            uintptr_t value = 0;
                            std::memcpy(&value, bytes + index, sizeof(value));
                            if (value != want)
                            {
                                continue;
                            }
                            void *candidate = reinterpret_cast<void *>(base + index);
                            int candidates = 0;
                            if (Validate(candidate, candidates))
                            {
                                if (g_log != nullptr)
                                {
                                    char text[160];
                                    std::snprintf(text, sizeof(text),
                                                  "model: graph at 0x%llX characters=%d",
                                                  static_cast<unsigned long long>(base + index),
                                                  candidates);
                                    g_log(text);
                                }
                                if (candidates <= 0)
                                {
                                    // Empty graph: keep sweeping for the live one.
                                    g_entities = nullptr;
                                    g_world = nullptr;
                                    continue;
                                }
                                g_scanning = false;
                                if (g_log != nullptr)
                                {
                                    char text[160];
                                    std::snprintf(text, sizeof(text),
                                                  "model: graph accepted (attempt %d)",
                                                  g_locateAttempts);
                                    g_log(text);
                                }
                                return true;
                            }
                        }
                    }
                }

                if (consumed == 0)
                {
                    break;
                }
                g_cursor = base + consumed;
            }

            if (g_cursor >= maximum)
            {
                // Whole address space swept without a hit. The battle graph is
                // only created when a match starts, so the sweep waits instead of
                // re-reading a few GB every frame while the menu is up.
                g_scanning = false;
                g_retryAtMs = GetTickCount64() + kScanRetryMs;
            }
            return false;
        }

        bool RefreshLocal()
        {
            g_local = nullptr;
            if (!Has(g_layout.player))
            {
                return false;
            }
            void *player = nullptr;
            if (!ReadField(g_entities, g_layout.player, player) || !Plausible(player))
            {
                return false;
            }
            if (!Has(g_layout.serverModel))
            {
                return false;
            }
            void *server = nullptr;
            if (!ReadField(player, g_layout.serverModel, server) || !Plausible(server))
            {
                return false;
            }
            g_local = server;
            return true;
        }

        bool ReadTransform(const void *character, Vec3 &out)
        {
            void *transform = nullptr;
            if (!ReadField(character, g_layout.transform, transform) || !Plausible(transform))
            {
                return false;
            }
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            if (!ReadField(transform, g_layout.transformX, x) ||
                !ReadField(transform, g_layout.transformY, y) ||
                !ReadField(transform, g_layout.transformZ, z))
            {
                return false;
            }
            constexpr float kMaxCoordinate = 1000000.0f;
            if (!(x > -kMaxCoordinate && x < kMaxCoordinate) ||
                !(y > -kMaxCoordinate && y < kMaxCoordinate) ||
                !(z > -kMaxCoordinate && z < kMaxCoordinate))
            {
                return false;
            }
            out = Vec3{ x, y, z };
            return true;
        }

        // Role / armor / health travel through IPrimitiveData holders: one
        // pointer to the holder, the payload at +0x18. A missing read leaves the
        // value at its "unknown" sentinel instead of inventing a zero.
        void ReadProfile(void *character, Character &out)
        {
            if (!Has(g_layout.data))
            {
                return;
            }
            void *data = nullptr;
            if (!ReadField(character, g_layout.data, data) || !Plausible(data))
            {
                return;
            }

            if (Has(g_layout.role))
            {
                void *common = nullptr;
                if (ReadField(data, g_layout.commonData, common) && Plausible(common))
                {
                    void *holder = nullptr;
                    if (ReadField(common, g_layout.role, holder) && Plausible(holder))
                    {
                        // The payload is a reference to the description object
                        // (PlayerRoleDescription : MyEnumItem : EnumItem :
                        // Identified), and its Id string is the role name. Reading
                        // the name beats mapping enum indices at the bench.
                        void *description = nullptr;
                        if (ReadField(holder, kPrimitiveValue, description) &&
                            Plausible(description) && Has(g_layout.roleId))
                        {
                            void *id = nullptr;
                            if (ReadField(description, g_layout.roleId, id) && Plausible(id))
                            {
                                const std::string text = g_resolver->ReadString(id, sizeof(out.role) - 1);
                                std::snprintf(out.role, sizeof(out.role), "%s", text.c_str());
                            }
                        }
                    }
                }
            }

            if (Has(g_layout.hasArmor))
            {
                void *publicData = nullptr;
                if (ReadField(data, g_layout.publicData, publicData) && Plausible(publicData))
                {
                    void *holder = nullptr;
                    if (ReadField(publicData, g_layout.hasArmor, holder) && Plausible(holder))
                    {
                        unsigned char value = 0;
                        if (ReadField(holder, kPrimitiveValue, value))
                        {
                            out.armor = value != 0 ? 1 : 0;
                        }
                    }
                }
            }

            if (Has(g_layout.health))
            {
                void *privateData = nullptr;
                if (ReadField(data, g_layout.privateData, privateData) && Plausible(privateData))
                {
                    void *holder = nullptr;
                    if (ReadField(privateData, g_layout.health, holder) && Plausible(holder))
                    {
                        int value = 0;
                        if (ReadField(holder, kPrimitiveValue, value))
                        {
                            out.health = value;
                        }
                    }
                }
            }
        }

        // Nicknames are the only thing the character object does not carry: the
        // game keeps them in the slot list, keyed by the same entity id. One
        // dictionary read per refresh serves every character, and the value is
        // matched by id, so a missing slot leaves the name empty instead of
        // borrowing a neighbour's.
        struct Nickname
        {
            int id = 0;
            char text[40] = {};
        };

        std::vector<Nickname> g_nicknames;
        unsigned long long g_nicknameTick = 0;
        // A name never changes mid-match, so a slow refresh is enough; the walk
        // itself is one bounded dictionary read.
        constexpr unsigned long long kNicknameTtlMs = 3000;

        void RefreshNicknames(unsigned long long nowMs)
        {
            if (!Has(g_layout.slotModel) || !Has(g_layout.slotByEntity) || !Has(g_layout.slotData) ||
                !Has(g_layout.slotNickname) || g_entities == nullptr)
            {
                return;
            }
            if (g_nicknameTick != 0 && nowMs >= g_nicknameTick && nowMs - g_nicknameTick < kNicknameTtlMs)
            {
                return;
            }
            g_nicknameTick = nowMs;

            void *slots = nullptr;
            void *dictionary = nullptr;
            if (!ReadField(g_entities, g_layout.slotModel, slots) || !Plausible(slots) ||
                !ReadField(slots, g_layout.slotByEntity, dictionary) || !Plausible(dictionary))
            {
                return;
            }
            int count = 0;
            if (!ReadField(dictionary, kDictCount, count) || count < 0 || count > kMaxCharacters)
            {
                return;
            }
            if (count == 0)
            {
                g_nicknames.clear();
                return;
            }
            void *entries = nullptr;
            if (!ReadField(dictionary, kDictEntries, entries) || !Plausible(entries))
            {
                return;
            }
            std::vector<unsigned char> raw(static_cast<size_t>(count) * kEntryStride);
            if (!ReadMem(static_cast<const char *>(entries) + kArrayData, raw.data(), raw.size()))
            {
                return;
            }

            std::vector<Nickname> list;
            list.reserve(static_cast<size_t>(count));
            for (int index = 0; index < count; ++index)
            {
                const unsigned char *entry = raw.data() + static_cast<size_t>(index) * kEntryStride;
                Nickname item;
                void *slot = nullptr;
                std::memcpy(&item.id, entry + kEntryKey, sizeof(item.id));
                std::memcpy(&slot, entry + kEntryValue, sizeof(slot));
                if (!Plausible(slot))
                {
                    continue;
                }
                void *data = nullptr;
                if (!ReadField(slot, g_layout.slotData, data) || !Plausible(data))
                {
                    continue;
                }
                void *holder = nullptr;
                if (!ReadField(data, g_layout.slotNickname, holder) || !Plausible(holder))
                {
                    continue;
                }
                void *text = nullptr;
                if (!ReadField(holder, kPrimitiveValue, text) || !Plausible(text))
                {
                    continue;
                }
                const std::string name = g_resolver->ReadString(text, sizeof(item.text) - 1);
                if (name.empty())
                {
                    continue;
                }
                std::snprintf(item.text, sizeof(item.text), "%s", name.c_str());
                list.push_back(item);
            }
            g_nicknames.swap(list);
        }

        const char *NicknameFor(int id)
        {
            for (const Nickname &item : g_nicknames)
            {
                if (item.id == id)
                {
                    return item.text;
                }
            }
            return nullptr;
        }

        bool Snapshot(unsigned long long nowMs)
        {
            // Role/armor pointer chains are the expensive part of a snapshot and
            // their values are stable, so they refresh on this timer.
            constexpr unsigned long long kProfileTtlMs = 1000;
            // A character whose transform stopped answering is dropped after
            // this long instead of keeping a box at its last position.
            constexpr unsigned long long kFeetTtlMs = 750;

            // Previous snapshot kept for the last-known-feet fallback below.
            const std::vector<Character> previous = g_characters;
            g_characters.clear();
            ++g_version;

            if (!ResolveLayout() || g_entities == nullptr)
            {
                return false;
            }

            RefreshLocal();

            // Same graph, one extra hop: the slot list that carries the names.
            RefreshNicknames(nowMs);

            void *dictionary = nullptr;
            if (!ReadField(g_world, g_layout.characters, dictionary) || !Plausible(dictionary))
            {
                g_entities = nullptr;
                return false;
            }

            int count = 0;
            if (!ReadField(dictionary, kDictCount, count) || count < 0 || count > kMaxCharacters)
            {
                g_entities = nullptr;
                return false;
            }
            if (count == 0)
            {
                return true;
            }

            void *entries = nullptr;
            if (!ReadField(dictionary, kDictEntries, entries) || !Plausible(entries))
            {
                return false;
            }

            // One transfer for the whole entry array: at most 512 * 0x18 = 12 KB.
            std::vector<unsigned char> raw(static_cast<size_t>(count) * kEntryStride);
            if (!ReadMem(static_cast<const char *>(entries) + kArrayData, raw.data(), raw.size()))
            {
                return false;
            }

            g_characters.reserve(static_cast<size_t>(count));
            for (int index = 0; index < count; ++index)
            {
                const unsigned char *entry = raw.data() + static_cast<size_t>(index) * kEntryStride;
                int key = 0;
                void *character = nullptr;
                std::memcpy(&key, entry + kEntryKey, sizeof(key));
                std::memcpy(&character, entry + kEntryValue, sizeof(character));
                if (!Plausible(character))
                {
                    continue;
                }

                Character record;
                record.character = character;
                record.id = key;
                if (!ReadTransform(character, record.feet))
                {
                    // A transform read that fails for one snapshot used to drop
                    // the character from the list entirely: that is exactly the
                    // "some players have no ESP" report, and it also made the
                    // boxes of everyone else shift a slot for a frame. Keep the
                    // last known feet instead — but only while they are fresh.
                    const Character *last = nullptr;
                    for (const Character &old : previous)
                    {
                        if (old.character == character)
                        {
                            last = &old;
                            break;
                        }
                    }
                    if (last == nullptr || last->feetTick == 0 ||
                        nowMs < last->feetTick || nowMs - last->feetTick > kFeetTtlMs)
                    {
                        continue;
                    }
                    record = *last;
                    record.id = key;
                }
                else
                {
                    record.feetTick = nowMs;
                    const Character *last = nullptr;
                    for (const Character &old : previous)
                    {
                        if (old.character == character)
                        {
                            last = &old;
                            break;
                        }
                    }
                    if (last != nullptr && last->profileTick != 0 &&
                        nowMs - last->profileTick < kProfileTtlMs)
                    {
                        std::memcpy(record.role, last->role, sizeof(record.role));
                        record.armor = last->armor;
                        record.health = last->health;
                        record.profileTick = last->profileTick;
                    }
                    else
                    {
                        ReadProfile(character, record);
                        record.profileTick = nowMs;
                    }
                }
                const char *nickname = NicknameFor(record.id);
                if (nickname != nullptr)
                {
                    std::snprintf(record.nickname, sizeof(record.nickname), "%s", nickname);
                }
                g_characters.push_back(record);
            }

            // The local player is excluded HERE, at the source: the pointer is
            // the game's own answer, not a distance guess, so no view of the own
            // character can slip back in between scans.
            if (g_local != nullptr)
            {
                g_characters.erase(
                    std::remove_if(g_characters.begin(), g_characters.end(),
                                   [](const Character &item) { return item.character == g_local; }),
                    g_characters.end());
            }

            return true;
        }
    } // namespace

    void Attach(Il2CppResolver *resolver, void (*log)(const char *message))
    {
        g_resolver = resolver;
        g_log = log;
    }

    void Reset()
    {
        g_layout = Layout{};
        g_battleModelClass = nullptr;
        g_entities = nullptr;
        g_world = nullptr;
        g_local = nullptr;
        g_characters.clear();
        g_nicknames.clear();
        g_nicknameTick = 0;
        g_entitiesKlass = nullptr;
        g_worldKlass = nullptr;
        g_scanning = false;
        g_cursor = 0;
        g_retryAtMs = 0;
        g_lastSnapshot = 0;
    }

    namespace
    {
        // The character list and the profiles are on the 33 ms snapshot budget,
        // but feet are the base every joint is drawn from. A snapshot-aged base
        // is a 10-20 cm lead or trail at 3-6 m/s - invisible in a box, obvious
        // on a 19-joint skeleton at 2 m. Only the transform is re-read here
        // (one pointer hop, twelve bytes per character), so the full snapshot
        // keeps its own timer and the profile hops stay where they were.
        void RefreshFeet(unsigned long long nowMs)
        {
            for (Character &record : g_characters)
            {
                if (record.character != nullptr && ReadTransform(record.character, record.feet))
                {
                    record.feetTick = nowMs;
                }
            }
        }
    }

    void Tick(unsigned long long nowMs)
    {
        g_requested = false;
        if (DisabledBySentinel())
        {
            std::snprintf(g_status, sizeof(g_status), "model: off (sentinel)");
            return;
        }
        if (!EnabledByFlag())
        {
            std::snprintf(g_status, sizeof(g_status), "model: off (no model flag file)");
            return;
        }
        // The caller treats "requested" as "the managed walk must never run":
        // while the graph is still being located every managed call is a chance
        // to deadlock the render thread (pid 19440 froze 40 s after inject,
        // beat file stopped inside the legacy FindObjectsOfType).
        g_requested = true;
        // Resolving is a burst of nine class lookups plus ~fourteen field
        // lookups, and it has to be throttled: when it fails, "resolved" stays
        // false and the retry used to run on EVERY Present frame — the render
        // thread ended up doing hundreds of metadata lookups a second and the
        // process died with a jump into the heap (0xC0000005, rip = page + 0xED7,
        // module=unmapped, two runs in a row).
        if (!g_layout.resolved)
        {
            static unsigned long long lastLayoutAttempt = 0;
            if (nowMs - lastLayoutAttempt < 2000)
            {
                return;
            }
            lastLayoutAttempt = nowMs;
            if (!ResolveLayout())
            {
                Log("model: layout failed | %s", g_status);
                return;
            }
        }
        if (g_entities == nullptr)
        {
            static unsigned long long lastScanLog = 0;
            if (nowMs - lastScanLog >= 1000)
            {
                lastScanLog = nowMs;
                Log("model: scanning, cursor=0x%llX attempt=%d",
                    static_cast<unsigned long long>(g_cursor), g_locateAttempts);
            }
            ScanSlice();
            if (g_entities == nullptr)
            {
                return;
            }
            // A fresh graph: force the snapshot below.
            g_lastSnapshot = 0;
            Log("model: graph accepted, entities=0x%llX world=0x%llX",
                static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(g_entities)),
                static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(g_world)));
        }

        if (nowMs - g_lastSnapshot < kSnapshotTtlMs && !g_characters.empty())
        {
            // Move the bodies, not the list: Sample() runs later in this same
            // tick, so the joints land on feet that are at most one frame old
            // instead of up to a snapshot old.
            RefreshFeet(nowMs);
            return;
        }
        g_lastSnapshot = nowMs;

        if (!Snapshot(nowMs))
        {
            // The graph died (battle ended, scene change): drop it and let the
            // locate scan run again. Never keep a stale list alive here, it
            // would draw boxes on characters that no longer exist.
            if (g_entities == nullptr)
            {
                g_characters.clear();
            }
            return;
        }

        static unsigned long long lastSnapshotLog = 0;
        if (nowMs - lastSnapshotLog >= 1000)
        {
            lastSnapshotLog = nowMs;
            Log("model: snapshot %zu characters, local %s", g_characters.size(),
                g_local != nullptr ? "known" : "unknown");
        }

        std::snprintf(g_status, sizeof(g_status), "model: %zu characters, local %s",
                      g_characters.size(), g_local != nullptr ? "known" : "unknown");
    }

    bool Ready()
    {
        return g_entities != nullptr && g_layout.resolved;
    }

    bool Requested()
    {
        return g_requested;
    }

    size_t Count()
    {
        return g_characters.size();
    }

    const Character *At(size_t index)
    {
        if (index >= g_characters.size())
        {
            return nullptr;
        }
        return &g_characters[index];
    }

    unsigned long long Version()
    {
        return g_version;
    }

    bool IsLocal(const void *character)
    {
        return character != nullptr && character == g_local;
    }

    void *Local()
    {
        return g_local;
    }

    const char *Status()
    {
        return g_status;
    }
}
