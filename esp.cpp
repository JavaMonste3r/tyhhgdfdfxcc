#include "esp.h"
#include "aim_math.h"
#include "esp_model.h"
#include "esp_pose.h"
#include "esp_pose_blend.h"
#include "esp_pulse.h"
#include "radar.h"
#include "imgui.h"

#include <Windows.h>
#include <timeapi.h> // timeBeginPeriod: the worker loop must see its 8 ms rate
#include <psapi.h>   // GetMappedFileNameW, for the module a page belongs to

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace omega::esp
{
    namespace
    {
        Il2CppResolver *g_resolver = nullptr;
        void (*g_log)(const char *) = nullptr;

        Options g_options{};

        // 0xRRGGBB from the options plus the per-element alpha the overlay has
        // always used. Kept in one place so a colour edit cannot leave one
        // element at the old alpha.
        ImU32 OptionColor(unsigned int rgb, int alpha)
        {
            return IM_COL32((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF, alpha);
        }

        // PlayerRoleDescription.<Id> as the game spells it. Robber and Prisoner
        // are the two criminal sides in this build; Settler/Citizen/Hubber and
        // the clan battles are all "not a criminal, not a cop". Anything else
        // stays Unknown and keeps the neutral box colour.
        RoleType RoleFromName(const char *role)
        {
            if (role == nullptr || role[0] == '\0')
            {
                return RoleType::Unknown;
            }
            const auto equals = [](const char *a, const char *b) {
                while (*a != '\0' && *b != '\0')
                {
                    char ca = *a++;
                    char cb = *b++;
                    if (ca >= 'A' && ca <= 'Z')
                    {
                        ca = static_cast<char>(ca - 'A' + 'a');
                    }
                    if (cb >= 'A' && cb <= 'Z')
                    {
                        cb = static_cast<char>(cb - 'A' + 'a');
                    }
                    if (ca != cb)
                    {
                        return false;
                    }
                }
                return *a == '\0' && *b == '\0';
            };
            if (equals(role, "Cop"))
            {
                return RoleType::Cop;
            }
            if (equals(role, "Robber") || equals(role, "Prisoner"))
            {
                return RoleType::Criminal;
            }
            if (equals(role, "Settler") || equals(role, "Citizen") || equals(role, "Hubber") ||
                equals(role, "ClanBattleMember"))
            {
                return RoleType::Innocent;
            }
            return RoleType::Unknown;
        }

        // Short label drawn above the nickname. The words are the ones the game
        // UI uses for the three sides.
        const char *RoleLabel(RoleType type)
        {
            switch (type)
            {
            case RoleType::Innocent:
                return "Невинный";
            case RoleType::Criminal:
                return "Криминал";
            case RoleType::Cop:
                return "Коп";
            default:
                return nullptr;
            }
        }

        unsigned int TypeColor(RoleType type, unsigned int fallback)
        {
            switch (type)
            {
            case RoleType::Innocent:
                return g_options.colorInnocent;
            case RoleType::Criminal:
                return g_options.colorCriminal;
            case RoleType::Cop:
                return g_options.colorCop;
            default:
                return fallback;
            }
        }

        // The anchor class is whatever MonoBehaviour each rendered player owns.
        // BattleCharacterCustomizationView is the character model view, so one
        // instance exists per character in the scene; it survives a rename of
        // the data models, which is why it is the default instead of a data class.
        char g_className[96] = "BattleCharacterCustomizationView";
        char g_classNamespace[128] = "Game.Battle.Entities.BattleCharacter.Customize.Views";
        float g_headOffset = 1.75f;

        // --- resolved managed entry points -----------------------------------
        // Resolved once per anchor-class change. MethodInfo pointers stay valid
        // for the life of the domain, so they are only dropped when the runtime
        // reports a failure on them.
        void *g_objectClass = nullptr;     // UnityEngine.Object, holds FindObjectsOfType
        void *g_cameraClass = nullptr;     // UnityEngine.Camera
        void *g_animatorClass = nullptr;   // UnityEngine.Animator, holds GetBoneTransform
        void *g_transformClass = nullptr;  // UnityEngine.Transform
        void *m_findObjects = nullptr;     // Object.FindObjectsOfType(Type)
        void *m_getMainCamera = nullptr;   // Camera.get_main()
        void *m_worldToScreen = nullptr;   // Camera.WorldToScreenPoint(Vector3)
        void *m_getTransform = nullptr;    // Component.get_transform()
        void *m_getPosition = nullptr;     // Transform.get_position()
        void *m_getName = nullptr;         // Object.get_name()
        void *m_getViewMatrix = nullptr;   // Camera.get_worldToCameraMatrix()
        void *m_getProjection = nullptr;   // Camera.get_projectionMatrix()
        // The _Injected variants (dump.cs:1107246, 1082050, 1082056) are
        // `static void m(IntPtr _unity_self, out T ret)`: they write the struct
        // straight into our storage and box nothing. Calling the managed
        // getter boxes one Vector3/Matrix4x4 per call, which is a heap
        // allocation per joint per read — the collector churn that made the
        // game slower the longer the esp ran.
        void *m_getPositionNative = nullptr;
        void *m_getViewMatrixNative = nullptr;
        void *m_getProjectionNative = nullptr;
        void *m_getBoneTransform = nullptr; // Animator.GetBoneTransform(HumanBodyBones)
        void *m_getIsHuman = nullptr;      // Animator.get_isHuman()
        void *m_getComponentInParent = nullptr; // Component.GetComponentInParent(Type,bool)
        void *m_getComponentInChildren = nullptr; // Component.GetComponentInChildren(Type,bool)
        void *m_getChildCount = nullptr;   // Transform.get_childCount()
        void *m_getChild = nullptr;        // Transform.GetChild(int)
        void *m_getParent = nullptr;       // Transform.get_parent()
        void *g_animatorType = nullptr;    // System.Type of UnityEngine.Animator
        void *g_camera = nullptr;          // last resolved main camera instance
        unsigned long long g_cameraTick = 0; // when g_camera was last looked up
        Vec3 g_cameraPosition{};           // eye position, derived from the view matrix
        bool g_cameraPositionValid = false;
        float g_cameraYawDeg = 0.0f;       // radar heading, 0 = +Z
        bool g_cameraYawValid = false;

        // Radar snapshot: what the ESP just resolved, copied once per frame so
        // the radar and the menu preview draw the same list.
        constexpr int kMaxRadarContacts = 64;
        RadarContact g_radarContacts[kMaxRadarContacts]{};
        int g_radarCount = 0;
        float g_radarEyeX = 0.0f;
        float g_radarEyeZ = 0.0f;
        float g_radarYaw = 0.0f;
        bool g_radarYawValid = false;

        // UnityEngine.Object keeps a native pointer in m_CachedPtr; it is nulled
        // when the object is destroyed while the managed wrapper survives until
        // the next GC. Calling into such a wrapper (or into a Transform of a
        // character that a scene load just destroyed) faults inside the runtime,
        // and the guard turns that into a five second ESP standstill. The offset
        // is read from the metadata at runtime, not hardcoded.
        size_t g_cachedPtrOffset = static_cast<size_t>(-1);

        // Whether the boxing-free path is trusted. It is proven once against the
        // managed getter (a runtime_invoke that failed to write the out parameter
        // back would silently return zeros), and dropped the moment it faults.
        bool g_positionNative = false;
        bool g_positionProbed = false;
        bool g_positionNativeTried = false;
        bool g_matrixNative = false;
        bool g_matrixProbed = false;
        bool g_matrixNativeTried = false;
        // The main camera is not parented under an Animator (CameraAnimator()
        // returns null), so the own body has to be found by intersecting the
        // camera's Transform ancestors with each slot's. Rebuilt once per scan.
        std::vector<void *> g_cameraChain;
        bool g_cameraChainValid = false;
        int g_selfByChain = 0;

        char g_resolvedKey[256] = "";      // namespace::name the cache belongs to
        char g_status[256] = "esp: idle";
        char g_debug[320] = "no walk yet";
        char g_aimDebug[192] = "aim: idle";

        // Ordered fallbacks: the ui field wins when it resolves, these cover a
        // rename of a single view. All three are one-instance-per-character
        // MonoBehaviours from dump.cs (BattleCharacterEntityView 376772,
        // BattleCharacterCustomizationView 8442, PlayerWeaponAnimationView 8773).
        struct Candidate
        {
            const char *namespaze;
            const char *name;
        };
        constexpr Candidate kCandidates[] = {
            { "Game.Battle.EntityModel.BattleCharacter.Views", "BattleCharacterEntityView" },
            { "Game.Battle.Entities.BattleCharacter.Customize.Views",
              "BattleCharacterCustomizationView" },
            { "Game.Battle.EntityModel.Player.Weapon", "PlayerWeaponAnimationView" },
        };

        void *g_anchor = nullptr;          // resolved class used by the walk
        const char *g_anchorSource = "none";

        // --- the cache ------------------------------------------------------
        // FindObjectsOfType is the one call that keeps killing the game: it
        // allocates a managed array, walks the scene graph and races the game
        // thread, and running it from Present every 500 ms was the crash in
        // %TEMP%\OmegaCore_fault.raw. So the component list is built rarely and
        // kept; between builds only Transform.get_position is read, which is a
        // native accessor on an object that is already known to exist.
        struct Slot
        {
            void *component = nullptr;  // the MonoBehaviour, the entity identity
            void *transform = nullptr;  // cached Component.get_transform()
            // Animator on the character root, resolved through
            // GetComponentInParent at scan time. Same pointer for every view the
            // character owns, which is what makes the self filter exact.
            void *animator = nullptr;
            bool animatorResolved = false;  // ResolveAnimator already ran
            bool bonesResolved = false;     // joint discovery finished (or given up)
            bool selfView = false;          // Transform chain overlaps the camera's
            // Which half of that test answered. The chain walks the camera's own
            // ancestors and is exact; the FollowCamera probe only says "some body
            // is inside 25 m of the eye", which in a crowd is several bodies at
            // once. Only the chain half may overrule a live latch.
            bool selfChain = false;
            // Once this slot was identified as part of the local player it stays
            // identified for the life of the scan. The nearest-body gate is
            // distance based and only fires while the own body is inside that
            // distance, so the box on the own character flickered back as soon
            // as the camera pulled away; the Transform pointer does not move.
            bool selfSticky = false;
            unsigned long long rigCooldown = 0; // no re-resolution before this tick
            // The mainRigBody Transform the joints were resolved under. Every
            // joint of a character hangs below exactly one of these nodes, and
            // on this rig the name table is fixed (skelet/жен.txt, муж.txt), so
            // a joint table filled from one subtree cannot borrow a neighbour's
            // arm. Kept through a rescan and validated with Alive().
            void *rigRoot = nullptr;
            // The node the name walk actually started from. A view component is
            // not always a descendant of the character root: a body parented to
            // a car seat, or a skin whose mainRigBody is a SIBLING of the view,
            // answers "not under slot.transform" for a rig that is perfectly its
            // own. Ownership is checked against this node instead, and the walk
            // keeps it so a rescan does not throw the rig away again.
            void *charRoot = nullptr;
            // Consecutive passes in which the torso chain came back longer than
            // a body can be. At 30 m/s a car moves 0.4-0.56 m per Unity physics
            // tick, so a single stretched read is a frame boundary, not another
            // character. Only a streak is treated as a foreign rig.
            unsigned char stretchStreak = 0;
            // Soft faults this slot has taken in a row. The cooldown grows
            // 1 s -> 2 s -> 5 s instead of parking the body for a full minute,
            // which is what kept a player who stepped out of a car boneless.
            unsigned char rigFaults = 0;
            // Player_Woman(Clone) ships without M_head_2_jnt: the head slot then
            // points at M_head_1_jnt, the skull base, and the crown has to be
            // derived along the spine or the head circle lands on the jaw.
            bool headIsBase = false;
            // Hips height above the feet anchor, in metres, from the last pose
            // that landed on this body. A standing body reads ~0.90 m and a body
            // seated in a car ~0.00 m, so `rig_miss: hipsDist=` tells a seated
            // player apart from a rig that simply failed. -1 when no pose ever
            // landed, which is a different case again.
            float hipsDist = -1.0f;
            int boneHits = 0;               // joints the rig actually exposed
            void *bone[kBoneCount]{};       // bone Transform pointers, by HumanBodyBones id
            Vec3 bonePos[kBoneCount]{};     // last read world position per joint
            bool boneOk[kBoneCount]{};      // that read succeeded
            // Consecutive passes in which this joint came back displaced
            // relative to the body's own feet. A read that fails the check is
            // rolled back to the previous position (the joint keeps being
            // drawn where its own body left it) and only a joint that repeats
            // the new place on the next pass is believed. Without the counter
            // a rollback would hold a genuinely re-parented joint forever.
            unsigned char boneHold[kBoneCount]{};
            // True for exactly one pass after the rig was (re)resolved. The
            // jump rule compares a joint against where THIS body last had it;
            // right after a resolve the previous positions belong to the old
            // node set, so every joint would "jump" by the distance between
            // the two rigs and drop the fresh rig again - a self-sustaining
            // loop that showed as 1849 "four or more joints changed owner" in
            // six minutes of the 22:15 run. The first pass after a resolve is
            // a baseline: it is measured, not judged.
            bool boneBaseline = false;
            // When the joints in bonePos were read, in microseconds. Doubles as
            // the capture identity: an unchanged stamp means BonePass has not
            // touched this body since the last frame, so its pose is the same
            // one and the blend must keep advancing instead of restarting.
            unsigned long long poseTick = 0;
            // Anchor the pose was read against, and the anchor of the latest
            // sample. The rig is drawn as a rigid body: joints are offset from
            // the anchor that was current when they were read, and re-based onto
            // the live anchor every frame. Without this each joint is an absolute
            // position from a slightly different moment and a character in a
            // moving car stutters while standing still far away looks smooth.
            Vec3 poseFeet{};
            Vec3 lastFeet{};
            bool haveFeet = false;
            unsigned long long feetTick = 0;
            char name[40] = "";
        };
        std::vector<Slot> g_slots;

        // Round-robin cursors for the two budgeted skeleton stages: rig
        // discovery is the expensive one, pose reads are the frequent one.
        size_t g_rigCursor = 0;
        size_t g_boneCursor = 0;
        unsigned long long g_bonePasses = 0;

        std::vector<Entity> g_entities;
        // --- thread-safe double buffer -------------------------------------
        // The worker thread builds g_entities; the render thread must not
        // touch it.  At the end of every ManagedTick the worker publishes a
        // snapshot under an exclusive lock; the render thread copies it under
        // a shared lock and then works on its own copy (g_renderEntities)
        // without holding any lock.  Draw() and Aim() iterate g_renderEntities
        // which is only ever touched by the render thread.
        SRWLOCK g_entityLock = SRWLOCK_INIT;
        std::vector<Entity> g_publishedEntities;   // written by worker under exclusive lock
        float g_publishedMvp[16]{};
        bool g_publishedMvpValid = false;
        std::vector<Entity> g_renderEntities;      // render-thread-local, never shared
        float g_renderMvp[16]{};
        bool g_renderMvpValid = false;
        // Snapshot version. The render thread copies the published list only
        // when this moved: Reproject writes into g_renderEntities every frame
        // (screen coords, filters), and re-copying a snapshot that did not
        // change threw that work away and restarted both follows.
        volatile LONG64 g_publishSeq = 0;          // bumped by the producer under g_entityLock
        LONG64 g_renderSeq = 0;                    // last version copied by the render thread
        // True when the il2cpp side runs on a pool thread and the projection on
        // the game's render thread. Declared here because Reproject() and Draw()
        // pick their matrix source from it.
        bool g_workerMode = true;
        ULONGLONG g_lastScan = 0;
        ULONGLONG g_platesTick = 0;       // own timer: never in a scan frame
        ULONGLONG g_lastSample = 0;
        bool g_forceScan = true;          // first tick after a class change
        // When the model graph first answered. The view-slot walk stays off until
        // this is a few seconds old: the deadlock (pid 19440) happened while the
        // graph was still being located, and a scan then is a managed call in a
        // streamed, half-built scene.
        ULONGLONG g_modelReadySince = 0;
        // The own body, latched for the life of the process. Every heuristic that
        // finds it (camera Animator, Transform chain, nearest-to-eye gate) can
        // miss for one sample, and the box on the own player came back exactly
        // then. Once found, the pointers themselves are the test.
        void *g_selfComponent = nullptr;
        void *g_selfAnimator = nullptr;
        // Set once the own body has been identified, and never re-decided until
        // the scene changes. Without it the search ran on every sample and the
        // answer moved: drive up to another player and the nearest-to-eye guess
        // switched to him, so the own box appeared and the neighbour's box
        // vanished - the exact report. The guess is a tie-breaker for the first
        // identification, not a per-frame decision.
        bool g_selfLocked = false;
        // Samples in a row where every scanned view of the latched body sat
        // beyond kSelfReleaseRadius. A latch is cleared only after this many,
        // because a single sample really can report the own body tens of metres
        // away (a fresh slot whose feet have not been sampled yet carries the
        // origin, and the eye can be 65 m from the origin). Releasing on one
        // sample un-filtered the own body for exactly one frame - the bones that
        // flashed on the player as he walked up to somebody - and the exact pass
        // re-latched in the same call, so the pair repeated every tick.
        constexpr int kSelfFarSamples = 30;
        int g_selfFarSamples = 0;
        // How much closer the nearest body has to be than the runner-up before a
        // distance guess is allowed to latch. Inside this band the guess is not
        // evidence - that is a crowd, and latching there would hide a stranger
        // for the rest of the match.
        constexpr float kSelfGateMargin = 1.5f;
        ULONGLONG g_scanFindCost = 0;     // FindObjectsOfType part of the last scan
        ULONGLONG g_scanTypeCost = 0;     // TypeObjectOf part of the last scan

        // Microsecond clock. GetTickCount64 advances in ~15.6 ms steps, so every
        // cost measured with it reads as 0, 15 or 31 and cannot locate a hotspot:
        // "scan: 16 ms, find 16" is really "somewhere below 15.6 ms".
        ULONGLONG NowMicroseconds()
        {
            static LARGE_INTEGER frequency{};
            if (frequency.QuadPart == 0)
            {
                QueryPerformanceFrequency(&frequency);
            }
            LARGE_INTEGER counter{};
            QueryPerformanceCounter(&counter);
            return frequency.QuadPart > 0
                       ? static_cast<ULONGLONG>(counter.QuadPart * 1000000ll / frequency.QuadPart)
                       : 0;
        }

        // Spatial medoid of a capture window: the real point with the smallest
        // sum of distances to the others. A per-axis median is beaten by two
        // bad captures in a row ([P, A, A] returns A) and can mix the X of one
        // pose with the Y of another, because each axis votes separately. The
        // medoid always returns one capture that actually happened, so three
        // pre-IK reads in a window of seven still return the post-IK point.
        Vec3 SpatialMedoid(const Vec3 *pts, int count)
        {
            if (count <= 0)
            {
                return Vec3{};
            }
            if (count <= 2)
            {
                return pts[count - 1];
            }
            int bestIdx = 0;
            float bestSum = 1.0e30f;
            for (int i = 0; i < count; ++i)
            {
                float sum = 0.0f;
                for (int j = 0; j < count; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }
                    sum += aim::Length(aim::Sub(pts[i], pts[j]));
                }
                if (sum < bestSum)
                {
                    bestSum = sum;
                    bestIdx = i;
                }
            }
            return pts[bestIdx];
        }
        // --- where the frame time goes ---------------------------------------
        // The cost line only said "the esp takes 2 ms": that number cannot tell a
        // slow managed walk from slow joint reads from slow drawing, and guessing
        // between them cost a whole round. Each stage stamps itself, the line
        // splits the frame, and a fix targets the part that actually moved.
        ULONGLONG g_costSampleUs = 0;
        ULONGLONG g_costBoneUs = 0;
        ULONGLONG g_costReprojUs = 0;
        ULONGLONG g_costDrawUs = 0;
        unsigned g_costFrames = 0;

        // Frame metrics, published by Reproject. They used to be counted at the
        // end of Sample(), but Sample() rebuilds every Entity from scratch, so
        // visible and onScreen were still false there and walk: printed
        // "0 projected, boneAge=0 ms" over a full screen of skeletons. Reproject
        // is the pass that fills them, so it is the pass that counts them, and
        // Sample only prints the last result.
        size_t g_lastProjected = 0;
        unsigned long long g_lastBoneAgeMs = 0;
        // Largest single-frame step of a drawn arm joint, in metres. This is the
        // only honest measure of the flip-flop the night spec is about: the
        // `bone jump` line is printed by BonePass on the raw capture, before the
        // medoid and the follow filter, so it stays loud even when the drawn
        // skeleton is already smooth. Read and reset by the `walk:` line.
        float g_drawnArmJumpM = 0.0f;
        // Largest distance between the model root and the rig root this sample,
        // in metres. The skeleton is drawn as `base + (joint - anchor)`: if the
        // base and the anchor are two different nodes, that difference is
        // applied to every joint as a shift, and its variation over an animation
        // reads as the skeleton leading or trailing the body. Reported by `walk:`
        // so the anchor choice is a number in the trace, not a belief.
        float g_anchorDelta = 0.0f;
        // First drawn body of the same pass, kept for the once-a-second
        // comparison against Camera.WorldToScreenPoint.
        bool g_probeValid = false;
        Vec3 g_probeFeet{};
        float g_probeFeetX = 0.0f;
        float g_probeFeetY = 0.0f;

        // Adds the lifetime of the scope to one accumulator, on every exit path
        // including the early returns: a stage measured only on the happy path
        // would report zero exactly when it is failing.
        struct CostScope
        {
            explicit CostScope(ULONGLONG &sink) : target(sink), start(NowMicroseconds()) {}
            ~CostScope() { target += NowMicroseconds() - start; }
            CostScope(const CostScope &) = delete;
            CostScope &operator=(const CostScope &) = delete;
            ULONGLONG &target;
            ULONGLONG start;
        };

        constexpr size_t kMaxEntities = 128;       // bounded: a corrupted count
                                                   // must not stall the render thread.

        constexpr size_t kMaxArgument = 1;
        // A predicted point further than this from the measured one is a torn
        // sample, not a running player; the lead is cut back to it.
        constexpr float kMaxLeadPixels = 200.0f;
        // Camera-space depth limits for drawing. Inside the near plane the
        // projection explodes, past the far plane the game does not render the
        // character either.
        constexpr float kNearPlane = 0.05f;
        constexpr float kFarPlane = 3000.0f;
        // Velocity older than this is stale (paused game, alt-tab, respawn) and
        // is zeroed instead of being extrapolated into a jump.
        constexpr ULONGLONG kVelocityDropMs = 3000;
        // --- skeleton cost budget -------------------------------------------
        // Every Transform.get_position used to be a managed call that boxes a
        // Vector3, i.e. a heap allocation; reading 19 joints of every character
        // on every sample allocated ~14k objects a second and the collector
        // stalled the game. The boxing-free path removed the allocation, and
        // the remaining cost is the invoke itself, so the budget is wider than
        // it was: six characters per tick keeps a 11-player match at ~30 Hz
        // pose reads instead of 11 Hz. Sixteen per tick plus a 4 ms budget keeps
        // an 11-player match at 60 Hz pose reads: the boxing-free path allocates
        // nothing, so the only cost is the invoke itself, and the old 1500 us cut
        // the pass after one body whenever the frame was busy. boneAge= in walk:
        // is the number this exists for.
        constexpr int kSlotsPerBonePass = 16;
        // Corner frequency of the anchor filter. With the capture back in front
        // of Reproject() the anchor is already this frame's, so the filter is
        // only polishing read noise: 6 ms replaces 93% of the value per 16 ms
        // frame and leaves a car at 20 m/s less than 0.12 m behind. 18 ms plus
        // the forward extrapolation was two predictions in series.
        constexpr float kSmoothTauMs = 6.0f;
        // Above this speed the filter is a pure lag and is bypassed: 35 m/s is
        // above any driven vehicle in the game, so only a teleport, a respawn or
        // a torn read skips the filter. 5 m/s used to be ordinary sprint, so the
        // filter never ran on exactly the frames the report is about.
        constexpr float kSmoothBypassSpeed = 35.0f;
        // Only the argument of blend::SelfCheck below: the look-ahead it belongs
        // to is no longer applied to drawn joints, but the same header also holds
        // the follow filter, and the self check proves the whole header.
        constexpr float kPoseMaxStep = 0.45f;
        // Corner frequency of the joint follow in Reproject. 22 ms erased a
        // capture step over three frames but on a running body it read as the
        // limbs freezing and then catching up: 22 ms of follow on top of a
        // captured pose that is already 8-30 ms old. A moving body now takes
        // 8 ms (alpha 0.86 at 16 ms), the medoid window is bypassed entirely so
        // there is nothing to freeze, and a standing body keeps the slow path.
        constexpr float kBoneSmoothTauMs = 8.0f;
        // Two captures of one suspicious jump have to land within this distance
        // of each other before the new place is believed. Two glitch frames of
        // the weapon race differ by ~0.9 m; one real pose repeats to a few cm.
        constexpr float kJumpAgreeMeters = 0.15f;
        // A per-frame jump larger than this is a rig change, a respawn or a
        // pooled slot handing over another body's joints, not motion: the
        // drawn point takes it directly instead of sliding across the map.
        constexpr float kBoneSnapStep = 0.30f;
        // Distance from the eye past which a latched self body is considered
        // handed over to someone else. The third-person spring measures 6.8 m
        // at rest in a live run; the wrong bodies a latch has sat on measured
        // 63.0 and 74.9 m after a pool reassignment. 40 m clears every
        // legitimate camera position and catches every observed failure.
        constexpr float kSelfReleaseRadius = 40.0f;
        // A body this far from the eye can never be the local player: the
        // third-person spring measured 2.9-6.8 m over every live run, the one
        // in a car included. The FollowCamera probe alone accepted a body whose
        // feet stood 61.1 m from the eye - the latch then flipped between
        // "latched 61.1 m" and "released 61.8 m" 57 times in one session, and
        // every flip moved the box between the enemy and the own character.
        constexpr float kSelfMaxLatchDistance = 25.0f;
        // A rig whose joints all failed to read is retried, but not on the very
        // next tick: a destroyed character would otherwise re-walk its whole
        // hierarchy on every frame.
        constexpr unsigned long long kRigRetryMs = 4000;
        // A humanoid avatar answers GetBoneTransform for every joint; below this
        // many answers the rig is generic and has to be read by bone names.
        constexpr int kHumanoidJointThreshold = 8;
        // Bounded hierarchy read. This runs once per character, inside one tick.
        // 160 was a ceiling that actually bit: on the second character the walk
        // stopped at 160 nodes and found 16/19 joints, missing three that hang
        // deeper in the transform tree. The walk is per character and cached, so
        // the higher bound costs nothing per frame.
        constexpr size_t kRigWalkMaxNodes = 350;
        // How often the main camera object is looked up again. Three live freezes
        // stopped between `bonepass done` and `camera+matrices done`, i.e. inside
        // the only managed call that runs in that gap; the camera component is
        // not recreated per frame, so the cached pointer plus a cheap native
        // pointer check is the honest answer and the invoke drops to 1 Hz.
        constexpr unsigned long long kCameraRefreshMs = 1000;

        // --- projection -----------------------------------------------------
        // Both matrices are Unity Matrix4x4 (16 floats, column-major, element
        // index col*4+row, confirmed against dump.cs:1098418 where m00 carries
        // [NativeName("m_Data[0]")] and m10 m_Data[1]).
        float g_mvp[16]{};               // projection * worldToCamera
        bool g_mvpValid = false;
        ULONGLONG g_mvpTick = 0;         // last successful matrix refresh

        // Captures kept per joint for the medoid. Seven covers a burst of
        // three pre-IK frames of the weapon-animation race; a moving body
        // drops to three so the filter adds no lag to a step.
        static constexpr int kBoneHistCap = 7;

        // Follow-filter state: one anchor low-pass and one joint follow per
        // body. It is owned by whoever projects the body. In present-thread
        // mode that is the History entry keyed on the view component; in
        // worker mode the projection runs on the render thread, which must not
        // touch g_history (the worker pushes into it and it reallocates), so
        // the render thread keeps its own copy in g_renderFilters.
        struct FilterState
        {
            Vec3 smoothFeet{};
            bool smoothInit = false;
            ULONGLONG smoothTick = 0;
            Vec3 bonesSmooth[kBoneCount]{};
            bool bonesSmoothValid = false;
            ULONGLONG bonesSmoothTick = 0;
            // Баг 1.1: дебаунс однокадровых выбросов (телепортов) костей.
            // Принимаем скачок > kBoneSnapStep только если он подтверждён
            // двумя последовательными ЗАХВАТАМИ (реальный респавн/смена позы).
            unsigned char boneJumpStreak[kBoneCount]{};
            // Capture id the streak was last advanced on. Counting render frames
            // made the streak depend on the frame rate: at 144 Hz a single
            // glitched capture was held for one frame and accepted on the next,
            // while a genuinely fast animation was frozen and then teleported.
            unsigned long long lastBoneTick = 0;
            // Capture history: the last kBoneHistCap captures of every joint,
            // kept as a ring (boneHistPos is the next write slot). The medoid
            // over this ring is what drops the pre-IK frames of the
            // weapon-animation race and the micro-shake of the head and torso.
            Vec3 boneHist[kBoneCount][kBoneHistCap]{};
            unsigned char boneHistCount[kBoneCount]{};
            unsigned char boneHistPos[kBoneCount]{};
            // Second capture of a suspicious jump, kept so the third one can be
            // compared against it: two glitch frames land in two different
            // places (hip and weapon), one real pose lands twice in the same.
            Vec3 bonePending[kBoneCount]{};
            bool bonePendingValid[kBoneCount]{};
            // The last box built from this body's own joints. A rig that drops
            // for a moment (a car door, a seat, one pooled read) used to fall
            // back to a fixed 0.42 x height and balloon around a body that had
            // just been measured at 0.28 x height. Two seconds of memory keeps
            // the box the size and height the body actually is.
            float lastWidthRatio = 0.0f;
            float lastHeadOffsetPx = 0.0f;
            ULONGLONG lastBoxTick = 0;
        };

        // Per-entity history keyed by the managed component pointer. Survives
        // the slot rebuild, so a pooled character keeps its velocity.
        struct History
        {
            void *key = nullptr;
            Vec3 feet{};
            Vec3 velocity{};
            ULONGLONG tick = 0;
            // Filter state lives here, keyed by the view component, and NOT in
            // Entity: Sample() rebuilds every Entity from scratch, and the
            // inheritance that used to carry these fields forward was keyed on
            // a previous entity whose boneTick was non-zero. A body whose bone
            // capture is stale therefore lost both filters on every read and
            // drew raw - sm= was byte-identical to raw= in 71 % of 65042 live:
            // lines. History is never rebuilt, so the filters now always run.
            FilterState filter;
            // QPC microseconds for the velocity estimate. GetTickCount64 steps
            // in 15.6 ms while Sample() runs every 8 ms, so a difference of two
            // positions was measured over 15 or 31 ms depending on where the
            // sample fell inside the tick: at 3 m/s the reported speed jumped
            // between 3.5 and 6.1 m/s, and that noise went straight into the
            // anchor extrapolation.
            ULONGLONG microTick = 0;
        };
        std::vector<History> g_history;

        // Render-thread filter state, worker mode only. Same fields as the
        // History copy so Reproject can drive both through one pointer.
        struct RenderFilterState
        {
            void *key = nullptr;
            FilterState filter;
        };
        std::vector<RenderFilterState> g_renderFilters;

        // Defined with the velocity tracker below; Reproject needs it first.
        History *FindHistory(void *key);
        FilterState *FindOrCreateRenderFilter(void *key);

        // Every joint vote belongs to one continuous pose. A teleport, a scene
        // change or a body-wide snap starts a new one, so the arm history and
        // the pending jump capture of the old pose must not be compared against
        // the new one - that comparison is how a respawn would be held back for
        // two captures.
        void ResetBoneHistory(FilterState &filter)
        {
            for (int bone = 0; bone < kBoneCount; ++bone)
            {
                filter.boneHistCount[bone] = 0;
                filter.boneHistPos[bone] = 0;
                filter.bonePendingValid[bone] = false;
                filter.boneJumpStreak[bone] = 0;
            }
        }

        void SetStatus(const char *text)
        {
            if (text == nullptr)
            {
                return;
            }
            std::snprintf(g_status, sizeof(g_status), "%s", text);
            if (g_log != nullptr)
            {
                g_log(text);
            }
        }

        // Crash-localising trace. Written with bare Win32 and no CRT buffering,
        // so the last line survives a hard fault and names the stage that died.
        // Change-only, and at most four lines a second: the stages alternate
        // every frame, and opening a file per stage made Present do three
        // CreateFile calls per frame — its own share of the stutter.
        void Trace(const char *stage)
        {
            static char previous[64] = "";
            static ULONGLONG lastWrite = 0;
            if (std::strcmp(stage, previous) == 0)
            {
                return;
            }
            const ULONGLONG now = GetTickCount64();
            const bool fault = std::strncmp(stage, "fault", 5) == 0;
            // Scan-stage diagnostics are one line per event, not per frame, and
            // they were being swallowed by the throttle: "scan: done", "plates:"
            // and "rig:" never appeared in a live trace that had 10 "scan: find".
            const bool event = std::strncmp(stage, "scan", 4) == 0 ||
                               std::strncmp(stage, "plates", 6) == 0 ||
                               std::strncmp(stage, "self", 4) == 0 ||
                               std::strncmp(stage, "rig", 3) == 0 ||
                               std::strncmp(stage, "tls", 3) == 0 ||
                               std::strncmp(stage, "worker", 6) == 0;
            if (!fault && !event && now - lastWrite < 250)
            {
                return;
            }
            lastWrite = now;
            std::snprintf(previous, sizeof(previous), "%s", stage);

            wchar_t path[MAX_PATH]{};
            if (GetTempPathW(MAX_PATH, path) == 0)
            {
                return;
            }
            lstrcatW(path, L"OmegaCore_esp.raw");
            HANDLE file = CreateFileW(path, FILE_APPEND_DATA,
                                      FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_ALWAYS,
                                      FILE_ATTRIBUTE_NORMAL, nullptr);
            if (file == INVALID_HANDLE_VALUE)
            {
                return;
            }
            DWORD written = 0;
            WriteFile(file, stage, static_cast<DWORD>(std::strlen(stage)), &written, nullptr);
            WriteFile(file, "\r\n", 2, &written, nullptr);
            CloseHandle(file);
        }

        // Unthrottled append to the same trace file. Used by the head-geometry
        // dump, which needs a full snapshot (all entities, one second) instead of
        // the "first changed line wins" throttle above.
        void TraceLine(const char *line)
        {
            wchar_t path[MAX_PATH]{};
            if (GetTempPathW(MAX_PATH, path) == 0)
            {
                return;
            }
            lstrcatW(path, L"OmegaCore_esp.raw");
            HANDLE file = CreateFileW(path, FILE_APPEND_DATA,
                                      FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_ALWAYS,
                                      FILE_ATTRIBUTE_NORMAL, nullptr);
            if (file == INVALID_HANDLE_VALUE)
            {
                return;
            }
            DWORD written = 0;
            WriteFile(file, line, static_cast<DWORD>(std::strlen(line)), &written, nullptr);
            WriteFile(file, "\r\n", 2, &written, nullptr);
            CloseHandle(file);
        }

        // The per-second live: trace writes to disk from the Present thread, so
        // it is armed only when %TEMP%\OmegaCore_livetrace exists. Read once.
        bool LiveTraceEnabled()
        {
            static int cached = -1;
            if (cached < 0)
            {
                wchar_t path[MAX_PATH]{};
                cached = 0;
                if (GetTempPathW(MAX_PATH, path) != 0)
                {
                    lstrcatW(path, L"OmegaCore_livetrace");
                    const DWORD attributes = GetFileAttributesW(path);
                    cached = (attributes != INVALID_FILE_ATTRIBUTES) ? 1 : 0;
                }
            }
            return cached == 1;
        }

        // Stage markers, gated by %TEMP%\OmegaCore_stage. A freeze inside the
        // present hook leaves no fault and no stack to read, so the last marker
        // published is the only evidence of which stage never returned.
        char g_lastStage[64] = "start";

        // The frame path publishes here and nowhere else. The earlier version
        // opened, appended and closed a file 6-7 times per frame (660-910
        // markers/s, 136 MB of trace); every observed stall happened only when
        // that writer was armed, and a stall inside the marker's own CreateFileW
        // is indistinguishable from a stall in the code between two markers.
        // This block is read from outside with one ReadProcessMemory, so the
        // frame path touches no file, no heap and no lock.
        struct StageSlot
        {
            volatile LONG seq;   // odd while a publish is in flight
            volatile LONG count;
            volatile DWORD tid;
            volatile ULONGLONG tick;
            char text[48];
            volatile LONG stageInside;   // 1 while MarkStage itself is running
            volatile LONG pad;
        };
        StageSlot g_stageSlot{};

        // A bounded copy instead of std::snprintf: the marker path runs 10-20
        // times a frame on the render thread, the format is always "%s", and
        // the CRT formatter brought its own 576-byte frame plus locale state
        // into a path that has no business touching either.
        void CopyLabel(char *out, size_t size, const char *text)
        {
            if (out == nullptr || size == 0)
            {
                return;
            }
            size_t i = 0;
            if (text != nullptr)
            {
                for (; i + 1 < size && text[i] != '\0'; ++i)
                {
                    out[i] = text[i];
                }
            }
            out[i] = '\0';
        }
        volatile LONG g_stageArmed = -1;   // -1 unknown, 0 off, 1 memory, 2 memory+file
        volatile LONG g_stageAddressLogged = 0;

        void MarkStage(const char *stage)
        {
            if (g_stageArmed < 0)
            {
                wchar_t path[MAX_PATH]{};
                LONG mode = 0;
                if (GetTempPathW(MAX_PATH, path) != 0)
                {
                    lstrcatW(path, L"OmegaCore_stage");
                    if (GetFileAttributesW(path) != INVALID_FILE_ATTRIBUTES)
                    {
                        mode = 1;
                        lstrcatW(path, L"file");
                        if (GetFileAttributesW(path) != INVALID_FILE_ATTRIBUTES)
                        {
                            mode = 2;
                        }
                    }
                }
                g_stageArmed = mode;
            }
            const LONG mode = g_stageArmed;
            if (mode <= 0)
            {
                return;
            }
            // Seqlock: the reader samples seq, copies, samples seq again and
            // retries on a mismatch or an odd value. No allocation, no I/O.
            InterlockedIncrement(&g_stageSlot.stageInside);
            InterlockedIncrement(&g_stageSlot.seq);
            g_stageSlot.tid = GetCurrentThreadId();
            g_stageSlot.tick = GetTickCount64();
            CopyLabel(g_stageSlot.text, sizeof(g_stageSlot.text), stage + 7);
            InterlockedIncrement(&g_stageSlot.count);
            InterlockedIncrement(&g_stageSlot.seq);
            if (InterlockedCompareExchange(&g_stageAddressLogged, 1, 0) == 0)
            {
                char line[128];
                std::snprintf(line, sizeof(line), "stage slot at 0x%llX (mode %ld)",
                              static_cast<unsigned long long>(
                                  reinterpret_cast<size_t>(&g_stageSlot)),
                              mode);
                TraceLine(line);
            }
            if (mode == 2)
            {
                TraceLine(stage);
            }
            CopyLabel(g_lastStage, sizeof(g_lastStage), stage + 7);
            InterlockedDecrement(&g_stageSlot.stageInside);
        }

        // Frames taken by the present hook. The watchdog watches this counter;
        // a stall means the render thread is wedged and its register set is the
        // only place the answer lives.
        volatile LONG64 g_frameSeq = 0;
        volatile LONG g_presentThread = 0;
        volatile LONG64 g_hangDumped = -1;

        // Inside the process the watchdog can do what the anti-cheat forbids from
        // outside: suspend our own render thread, take its context and walk the
        // raw stack for return addresses. Each address is resolved against the
        // image ranges of this process, so a frame reads `gameassembly.dll+0x...`
        // and can be looked up in the il2cpp dump by RVA.
        void DumpHang(const char *reason)
        {
            const DWORD tid = static_cast<DWORD>(g_presentThread);
            char head[256];
            std::snprintf(head, sizeof(head), "HANG %s stage=%s tid=%lu", reason, g_lastStage,
                          static_cast<unsigned long>(tid));
            TraceLine(head);
            if (tid == 0)
            {
                return;
            }
            HANDLE thread = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT |
                                           THREAD_QUERY_INFORMATION,
                                       FALSE, tid);
            if (thread == nullptr)
            {
                char line[128];
                std::snprintf(line, sizeof(line), "HANG open failed win32 %lu", GetLastError());
                TraceLine(line);
                return;
            }
            if (SuspendThread(thread) == static_cast<DWORD>(-1))
            {
                char line[128];
                std::snprintf(line, sizeof(line), "HANG suspend failed win32 %lu",
                              GetLastError());
                TraceLine(line);
                CloseHandle(thread);
                return;
            }
            CONTEXT context{};
            context.ContextFlags = CONTEXT_CONTROL | CONTEXT_INTEGER;
            if (GetThreadContext(thread, &context))
            {
                char line[192];
                std::snprintf(line, sizeof(line), "HANG rip=%016llX rsp=%016llX rbp=%016llX",
                              static_cast<unsigned long long>(context.Rip),
                              static_cast<unsigned long long>(context.Rsp),
                              static_cast<unsigned long long>(context.Rbp));
                TraceLine(line);

                // Image ranges, merged per file. VirtualQuery in this process is
                // free and sees every module, including the Unity player.
                struct Range
                {
                    unsigned long long base;
                    unsigned long long end;
                    char name[48];
                };
                Range ranges[64]{};
                int rangeCount = 0;
                unsigned long long cursor = 0;
                for (int guard = 0; guard < 60000 && rangeCount < 64; ++guard)
                {
                    MEMORY_BASIC_INFORMATION info{};
                    if (VirtualQuery(reinterpret_cast<const void *>(cursor), &info,
                                     sizeof(info)) == 0)
                    {
                        break;
                    }
                    const unsigned long long base = reinterpret_cast<unsigned long long>(
                        info.BaseAddress);
                    const unsigned long long size = info.RegionSize;
                    if (info.Type == MEM_IMAGE && (info.State & MEM_COMMIT) != 0)
                    {
                        wchar_t path[MAX_PATH]{};
                        if (GetMappedFileNameW(GetCurrentProcess(), info.BaseAddress, path,
                                               MAX_PATH) > 0)
                        {
                            const wchar_t *slash = wcsrchr(path, L'\\');
                            const wchar_t *leaf = slash != nullptr ? slash + 1 : path;
                            char narrow[48]{};
                            WideCharToMultiByte(CP_ACP, 0, leaf, -1, narrow, sizeof(narrow) - 1,
                                                nullptr, nullptr);
                            for (char &c : narrow)
                            {
                                if (c >= 'A' && c <= 'Z')
                                {
                                    c = static_cast<char>(c - 'A' + 'a');
                                }
                            }
                            Range &last = ranges[rangeCount > 0 ? rangeCount - 1 : 0];
                            if (rangeCount > 0 && last.end == base &&
                                std::strcmp(last.name, narrow) == 0)
                            {
                                last.end = base + size;
                            }
                            else
                            {
                                Range &fresh = ranges[rangeCount++];
                                fresh.base = base;
                                fresh.end = base + size;
                                std::snprintf(fresh.name, sizeof(fresh.name), "%s", narrow);
                            }
                        }
                    }
                    if (base + size <= cursor)
                    {
                        break;
                    }
                    cursor = base + size;
                }

                // Return addresses on the stack, in the order a human reads a
                // backtrace: from the frame the thread is in, upward.
                const size_t span = 0x10000;
                unsigned char *stack = static_cast<unsigned char *>(VirtualAlloc(
                    nullptr, span, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
                if (stack != nullptr)
                {
                    SIZE_T got = 0;
                    if (ReadProcessMemory(GetCurrentProcess(),
                                          reinterpret_cast<const void *>(context.Rsp), stack,
                                          span, &got) &&
                        got >= 8)
                    {
                        unsigned long long previous = 0;
                        int printed = 0;
                        for (size_t offset = 0; offset + 8 <= got && printed < 24;
                             offset += 8)
                        {
                            unsigned long long value = 0;
                            std::memcpy(&value, stack + offset, sizeof(value));
                            if (value == previous)
                            {
                                continue;
                            }
                            const Range *hit = nullptr;
                            for (int i = 0; i < rangeCount; ++i)
                            {
                                if (value >= ranges[i].base && value < ranges[i].end)
                                {
                                    hit = &ranges[i];
                                    break;
                                }
                            }
                            if (hit == nullptr)
                            {
                                continue;
                            }
                            previous = value;
                            char frame[192];
                            std::snprintf(frame, sizeof(frame), "HANG   +%04llX %s+%llX",
                                          static_cast<unsigned long long>(offset), hit->name,
                                          value - hit->base);
                            TraceLine(frame);
                            ++printed;
                        }
                    }
                    VirtualFree(stack, 0, MEM_RELEASE);
                }
            }
            ResumeThread(thread);
            CloseHandle(thread);
        }

        // Longjmp out of a managed frame leaves the il2cpp runtime inconsistent,
        // so after a caught fault the walk backs off instead of hammering the
        // same broken state through a scene transition.
        ULONGLONG g_faultUntil = 0;
        // Consecutive caught faults. A flat 800 ms stand-down was not enough to
        // let a broken managed state settle: the trace of a live session carried
        // 411 scan attempts against 11 completions, i.e. every retry landed in
        // the same fault and the storm ended in a frozen process. Each repeat
        // doubles the wait instead, and a clean scan resets it.
        int g_faultStreak = 0;
        // Set while a call is allowed to fail without standing the whole ESP
        // down. The skeleton is optional per character; a dead Animator there
        // must cost that character's rig, not thirteen seconds of everything.
        bool g_softInvoke = false;
        bool g_softFaulted = false;
        // Set by Hold(); the walk stands still while the game swaps ui state.
        ULONGLONG g_holdUntil = 0;
        // Name of the managed call in flight. A caught exception is the one
        // failure that cannot be debugged from the outside, so the stage that
        // raised it is written next to the backoff line.
        const char *g_stage = "idle";
        // The hierarchy walk is the only stage that calls Object.get_name on
        // hundreds of transforms per character, and a destroyed node there takes
        // the whole managed thread down. Two faults out of it are enough: the
        // skeleton falls back and the boxes, names and distance keep running.
        // The rig walk is parked per character (Slot::rigCooldown), never for the
        // whole session: characters keep spawning on a map this size, and a global
        // switch left every newcomer without a skeleton.
        int g_scanFaults = 0;
        // A scan that keeps faulting is paused, not switched off: two faults
        // thirty minutes apart used to end scanning for the whole session, and
        // on a map this size newcomers then never got a skeleton again. The
        // cooldown lets the scan heal by itself in 15 s.
        constexpr ULONGLONG kScanFaultCooldownMs = 15000;
        ULONGLONG g_scanCooldownUntil = 0;

        void MarkFault()
        {
            // Fixed 1.2 s, deliberately NOT exponential. The escalating backoff
            // was added on the theory that a longer pause lets a broken managed
            // state settle; the trace refuted it — a live session kept raising at
            // the same rip every scan, and the growth to 12.8 s is exactly what
            // turned one fault per scan into "the esp updates every 13 seconds".
            // A pause cannot cure a dead pointer, it only hides the esp.
            g_faultStreak = 0;
            g_faultUntil = GetTickCount64() + 1200;
            if (g_resolver != nullptr)
            {
                // The thread's il2cpp state did not survive the longjmp: it has
                // to be re-attached or every later invoke returns null.
                g_resolver->InvalidateThread();
            }
            // The slot list is NOT dropped. Wiping it here is what made the esp
            // blink out for whole seconds: every fault during a scene load took
            // the boxes away, the following scan re-walked every rig by name,
            // faulted again, and the cycle never settled. The list is the best
            // information available; slots whose transform died are skipped by
            // the Alive() guard in Sample().
            Trace("fault: backoff");
            // Which managed call raised: a bare backoff line cannot tell a bad
            // string read from a non-humanoid GetBoneTransform.
            char stage[96];
            std::snprintf(stage, sizeof(stage), "fault: at %s", g_stage);
            Trace(stage);
            // A fault inside the hierarchy walk parks only THIS slot: ResolveRig
            // catches it softly and sets a per-character cooldown. There is no
            // session-wide switch here on purpose — the map is big and characters
            // keep spawning, so one bad rig must never cost everyone else their
            // skeleton.
            if (lstrcmpA(g_stage, "scan: find") == 0)
            {
                ++g_scanFaults;
                if (g_scanFaults >= 2)
                {
                    g_scanCooldownUntil = GetTickCount64() + kScanFaultCooldownMs;
                    g_scanFaults = 0;
                    char line[96];
                    std::snprintf(line, sizeof(line),
                                  "scan: cooling down %llu s after repeated faults",
                                  static_cast<unsigned long long>(kScanFaultCooldownMs / 1000));
                    Trace(line);
                }
            }
        }

        // One managed call with a single optional argument. The arguments array
        // is a local: il2cpp_runtime_invoke does not retain it.
        void *Invoke1(void *method, void *instance, void *argument, bool *raised = nullptr)
        {
            if (g_resolver == nullptr || method == nullptr)
            {
                return nullptr;
            }
            bool faulted = false;
            bool managed = false;
            void *arguments[kMaxArgument] = { argument };
            void *result = g_resolver->Invoke(method, instance, arguments, &faulted, &managed);
            if (faulted)
            {
                if (!managed)
                {
                    g_softFaulted = true;
                    if (!g_softInvoke)
                    {
                        MarkFault();
                    }
                    if (raised != nullptr)
                    {
                        *raised = true;
                    }
                }
            }
            return result;
        }

        // Two arguments. GetComponentInParent(Type, bool) is the only caller:
        // the managed signature takes the type handle plus an include-inactive
        // flag, and the flag must be a value, not a boxed value.
        void *Invoke2(void *method, void *instance, void *a0, void *a1)
        {
            if (g_resolver == nullptr || method == nullptr)
            {
                return nullptr;
            }
            bool faulted = false;
            bool managed = false;
            void *arguments[2] = { a0, a1 };
            void *result = g_resolver->Invoke(method, instance, arguments, &faulted, &managed);
            if (faulted)
            {
                if (!managed)
                {
                    g_softFaulted = true;
                    if (!g_softInvoke)
                    {
                        MarkFault();
                    }
                }
                return nullptr;
            }
            return result;
        }

        // Drops every cached entry point. Called when the anchor class changes or
        // when a cached MethodInfo turns out to be stale after a scene reload.
        void ResetCache()
        {
            g_objectClass = nullptr;
            g_cameraClass = nullptr;
            g_transformClass = nullptr;
            m_findObjects = nullptr;
            m_getMainCamera = nullptr;
            m_worldToScreen = nullptr;
            m_getTransform = nullptr;
            m_getPosition = nullptr;
            m_getPositionNative = nullptr;
            m_getViewMatrixNative = nullptr;
            m_getProjectionNative = nullptr;
            g_positionNative = false;
            g_positionProbed = false;
            g_positionNativeTried = false;
            g_matrixNative = false;
            g_matrixProbed = false;
            g_matrixNativeTried = false;
            g_cameraChain.clear();
            g_cameraChainValid = false;
            m_getName = nullptr;
            m_getViewMatrix = nullptr;
            m_getProjection = nullptr;
            m_getBoneTransform = nullptr;
            m_getIsHuman = nullptr;
            m_getComponentInParent = nullptr;
            m_getComponentInChildren = nullptr;
            m_getChildCount = nullptr;
            m_getChild = nullptr;
            m_getParent = nullptr;
            g_animatorType = nullptr;
            g_camera = nullptr;
            g_cameraPositionValid = false;
            g_mvpValid = false;
            g_anchor = nullptr;
            g_anchorSource = "none";
            g_resolvedKey[0] = '\0';
            g_slots.clear();
            g_history.clear();
            g_forceScan = true;
            g_modelReadySince = 0;
        }

        // Everything that depends only on the metadata: resolve once, keep for
        // the session. A failure here is a wrong class name, not a runtime fault,
        // so it is reported rather than retried on every frame.
        bool Resolve(void *anchor)
        {
            if (g_resolver == nullptr || anchor == nullptr)
            {
                return false;
            }

            if (g_objectClass == nullptr)
            {
                g_objectClass = g_resolver->FindClassAnyImage("UnityEngine", "Object");
            }
            if (g_cameraClass == nullptr)
            {
                g_cameraClass = g_resolver->FindClassAnyImage("UnityEngine", "Camera");
            }
            if (g_animatorClass == nullptr)
            {
                g_animatorClass = g_resolver->FindClassAnyImage("UnityEngine", "Animator");
            }
            if (g_objectClass == nullptr || g_cameraClass == nullptr)
            {
                SetStatus("esp: UnityEngine.Object/Camera not found");
                return false;
            }
            if (g_cachedPtrOffset == static_cast<size_t>(-1))
            {
                g_cachedPtrOffset = g_resolver->FieldOffset(g_objectClass, "m_CachedPtr");
            }

            if (m_findObjects == nullptr)
            {
                m_findObjects = g_resolver->Method(g_objectClass, "FindObjectsOfType", 1);
            }
            if (m_getMainCamera == nullptr)
            {
                m_getMainCamera = g_resolver->Method(g_cameraClass, "get_main", 0);
            }
            if (m_getViewMatrix == nullptr)
            {
                m_getViewMatrix =
                    g_resolver->Method(g_cameraClass, "get_worldToCameraMatrix", 0);
            }
            if (m_getProjection == nullptr)
            {
                m_getProjection = g_resolver->Method(g_cameraClass, "get_projectionMatrix", 0);
            }
            if (m_getViewMatrixNative == nullptr)
            {
                // Optional: the managed getter above is a complete fallback, so
                // a miss here costs allocations, not the esp.
                m_getViewMatrixNative =
                    g_resolver->Method(g_cameraClass, "get_worldToCameraMatrix_Injected", 2);
            }
            if (m_getProjectionNative == nullptr)
            {
                m_getProjectionNative =
                    g_resolver->Method(g_cameraClass, "get_projectionMatrix_Injected", 2);
            }
            if (m_worldToScreen == nullptr)
            {
                // Diagnostics only: one call a second, checked against the
                // native projection to prove the matrix path agrees with Unity.
                m_worldToScreen = g_resolver->Method(g_cameraClass, "WorldToScreenPoint", 1);
            }
            if (m_getTransform == nullptr)
            {
                // Declared on Component; Method() walks the base chain.
                m_getTransform = g_resolver->Method(anchor, "get_transform", 0);
            }
            if (m_getComponentInParent == nullptr)
            {
                // Optional: only the skeleton and the exact self filter need it.
                m_getComponentInParent = g_resolver->Method(anchor, "GetComponentInParent", 2);
            }
            if (m_getComponentInChildren == nullptr)
            {
                m_getComponentInChildren = g_resolver->Method(anchor, "GetComponentInChildren", 2);
            }
            if (m_getBoneTransform == nullptr && g_animatorClass != nullptr)
            {
                m_getBoneTransform = g_resolver->Method(g_animatorClass, "GetBoneTransform", 1);
                // Asking a non-humanoid animator for a bone throws inside the
                // runtime, and a caught exception stands the whole ESP down for
                // five seconds — that was the periodic freeze in the log. The
                // rig kind is therefore checked before the call.
                m_getIsHuman = g_resolver->Method(g_animatorClass, "get_isHuman", 0);
            }
            if (g_animatorType == nullptr && g_animatorClass != nullptr)
            {
                g_animatorType = g_resolver->TypeObjectOf(g_animatorClass);
            }
            if (m_getName == nullptr)
            {
                // Optional: only feeds the name label. A null here costs the
                // label, nothing else, so it is not part of the gate below.
                m_getName = g_resolver->Method(g_objectClass, "get_name", 0);
            }

            if (m_findObjects == nullptr || m_getMainCamera == nullptr ||
                m_getViewMatrix == nullptr || m_getProjection == nullptr ||
                m_getTransform == nullptr)
            {
                char text[256];
                std::snprintf(text, sizeof(text),
                              "esp: missing managed entry (find=%d cam=%d view=%d proj=%d tr=%d)",
                              m_findObjects != nullptr, m_getMainCamera != nullptr,
                              m_getViewMatrix != nullptr, m_getProjection != nullptr,
                              m_getTransform != nullptr);
                SetStatus(text);
                return false;
            }
            // Arm the boxing-free matrix path once. A mismatch during the probe
            // disarms it for the session, so this never becomes a retry loop.
            if (!g_matrixNativeTried)
            {
                g_matrixNativeTried = true;
                g_matrixNative =
                    m_getViewMatrixNative != nullptr && m_getProjectionNative != nullptr;
            }
            return true;
        }

        bool ResolveAnchor()
        {
            if (g_resolver == nullptr)
            {
                SetStatus("esp: no resolver attached");
                return false;
            }
            // The bridge is only needed by the esp/aim path, so it is brought up
            // here on first use instead of at bootstrap.
            if (!g_resolver->Ready() && !g_resolver->Attach())
            {
                char text[256];
                std::snprintf(text, sizeof(text), "esp: il2cpp attach failed: %s",
                              g_resolver->LastError());
                SetStatus(text);
                return false;
            }
            // Present runs on the render thread, which the il2cpp GC has never
            // seen; managed calls from it allocate unregistered.
            if (!g_resolver->AttachThread())
            {
                SetStatus("esp: thread attach failed");
                return false;
            }

            char key[256];
            std::snprintf(key, sizeof(key), "%s::%s", g_classNamespace, g_className);
            if (std::strcmp(key, g_resolvedKey) != 0)
            {
                ResetCache();
                std::snprintf(g_resolvedKey, sizeof(g_resolvedKey), "%s", key);
            }
            else if (g_anchor != nullptr)
            {
                // Same class as last frame: the metadata lookups are already
                // done and the class handle does not move. This is the per-frame
                // path, so it must not touch the assembly list at all.
                return true;
            }

            void *anchor = g_resolver->FindClassAnyImage(g_classNamespace, g_className);
            if (anchor != nullptr)
            {
                g_anchor = anchor;
                g_anchorSource = "ui";
            }
            else
            {
                // The ui field is a single guess; a game patch can rename one
                // view without touching the rest of the hierarchy, so a short
                // ordered list of character views is tried before giving up.
                for (const Candidate &candidate : kCandidates)
                {
                    void *found = g_resolver->FindClassAnyImage(candidate.namespaze,
                                                                candidate.name);
                    if (found != nullptr)
                    {
                        g_anchor = found;
                        g_anchorSource = candidate.name;
                        break;
                    }
                }
            }

            if (g_anchor == nullptr)
            {
                char text[256];
                std::snprintf(text, sizeof(text), "esp: class not found %s", key);
                SetStatus(text);
                return false;
            }
            return Resolve(g_anchor);
        }

        // UnityEngine.Object keeps a native pointer in m_CachedPtr; it is nulled
        // when the object is destroyed while the managed wrapper survives until
        // the next GC. Reading it before a call is what keeps a scene load from
        // faulting inside the runtime.
        // The native object behind a managed wrapper. The _Injected accessors take
        // exactly this pointer as their first argument (Transform.get_position
        // passes `[this+0x10]` to the cached native getter), so one read here is
        // what turns a boxed managed call into a direct native one.
        bool NativePointer(void *object, void *&native)
        {
            native = nullptr;
            if (object == nullptr || g_cachedPtrOffset == static_cast<size_t>(-1))
            {
                return false;
            }
            const uintptr_t address = reinterpret_cast<uintptr_t>(object) + g_cachedPtrOffset;
            if (ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<LPCVOID>(address), &native,
                                  sizeof(native), nullptr) == FALSE)
            {
                return false;
            }
            return native != nullptr;
        }

        bool Alive(void *object)
        {
            if (object == nullptr)
            {
                return false;
            }
            if (g_cachedPtrOffset == static_cast<size_t>(-1))
            {
                // No offset means no way to tell; the caller's guard still holds.
                return true;
            }
            void *native = nullptr;
            const uintptr_t address = reinterpret_cast<uintptr_t>(object) + g_cachedPtrOffset;
            if (ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<LPCVOID>(address), &native,
                                  sizeof(native), nullptr) == FALSE)
            {
                return false;
            }
            return native != nullptr;
        }

        // Managed getter: one invoke plus one boxed Vector3. The reference the
        // boxing-free path is proven against, and the fallback when it is off.
        bool TransformPositionBoxed(void *transform, Vec3 &out)
        {
            if (transform == nullptr || m_getPosition == nullptr || !Alive(transform))
            {
                return false;
            }
            void *boxed = Invoke1(m_getPosition, transform, nullptr);
            if (boxed == nullptr)
            {
                return false;
            }
            return g_resolver->ReadVec3(boxed, out);
        }

        // Same value, no boxed Vector3: the _Injected variant writes into a local.
        bool TransformPositionNative(void *transform, Vec3 &out)
        {
            if (transform == nullptr || m_getPositionNative == nullptr)
            {
                return false;
            }
            void *native = nullptr;
            if (!NativePointer(transform, native))
            {
                return false;
            }
            Vec3 value{};
            bool faulted = false;
            void *arguments[2] = { &native, &value };
            g_resolver->Invoke(m_getPositionNative, nullptr, arguments, &faulted);
            if (faulted)
            {
                // One fault is enough: the runtime disagreed with the ABI this
                // path assumes, so every later read goes through the getter.
                g_positionNative = false;
                m_getPositionNative = nullptr;
                Trace("pos: injected off");
                return false;
            }
            out = value;
            return true;
        }

        bool TransformPosition(void *transform, Vec3 &out)
        {
            if (m_getPositionNative != nullptr && g_positionNative)
            {
                Vec3 value{};
                if (TransformPositionNative(transform, value))
                {
                    if (!g_positionProbed)
                    {
                        // Proven once against the managed getter, on the same
                        // transform, so a silently empty result cannot reach the
                        // projection.
                        Vec3 reference{};
                        if (TransformPositionBoxed(transform, reference))
                        {
                            const float dx = value.x - reference.x;
                            const float dy = value.y - reference.y;
                            const float dz = value.z - reference.z;
                            g_positionProbed = true;
                            if (dx * dx + dy * dy + dz * dz > 0.01f)
                            {
                                g_positionNative = false;
                                Trace("pos: injected mismatch");
                                out = reference;
                                return true;
                            }
                            // The cheap path is what the bone budget spends its
                            // microseconds on: one line so the log says which
                            // reader a session is actually paying for.
                            Trace("pos: injected active");
                        }
                    }
                    out = value;
                    return true;
                }
            }
            return TransformPositionBoxed(transform, out);
        }

        // Column-major multiply, the order Unity's Matrix4x4 uses:
        // out = a * b, element [col*4+row].
        void Multiply(const float *a, const float *b, float *out)
        {
            for (int column = 0; column < 4; ++column)
            {
                for (int row = 0; row < 4; ++row)
                {
                    float sum = 0.0f;
                    for (int k = 0; k < 4; ++k)
                    {
                        sum += a[k * 4 + row] * b[column * 4 + k];
                    }
                    out[column * 4 + row] = sum;
                }
            }
        }

        bool RefreshCamera(bool force = false)
        {
            if (m_getMainCamera == nullptr)
            {
                return false;
            }
            // The camera component survives frame to frame; the lookup is a
            // managed invoke on the render thread and the only one left in this
            // stage, so it is reused while the native object behind it is alive.
            const ULONGLONG nowTick = GetTickCount64();
            if (!force && g_camera != nullptr && nowTick >= g_cameraTick &&
                nowTick - g_cameraTick < kCameraRefreshMs)
            {
                void *native = nullptr;
                if (NativePointer(g_camera, native))
                {
                    return true;
                }
            }
            MarkStage("stage: camera invoke");
            void *camera = g_resolver->Invoke(m_getMainCamera, nullptr, nullptr, nullptr);
            g_cameraTick = nowTick;
            MarkStage("stage: camera invoke done");
            if (camera == nullptr)
            {
                g_camera = nullptr;
                g_mvpValid = false;
                g_cameraPositionValid = false;
                return false;
            }
            g_camera = camera;
            return true;
        }

        // The Animator that owns the main camera. The third-person rig is
        // parented under the character it follows, so walking up from the camera
        // component lands on that character's skeleton root. One call per sample.
        void *CameraAnimator()
        {
            if (g_camera == nullptr || m_getComponentInParent == nullptr ||
                g_animatorType == nullptr)
            {
                return nullptr;
            }
            bool includeInactive = true;
            return Invoke2(m_getComponentInParent, g_camera, g_animatorType, &includeInactive);
        }

        // One 4x4 matrix out of the camera, boxed-free when the injected variant
        // is proven, through the managed getter otherwise. Both are 64 bytes.
        bool ReadMatrix(void *injected, void *managed, float *out)
        {
            // No injected variant: keep the matrices we already have rather than
            // calling the boxed getter. That call allocates a Matrix4x4 on a
            // thread il2cpp did not create, which is the freeze documented above
            // ProbeMatrices. A stale matrix for a few frames is invisible; a
            // wedged render thread is not.
            if (injected != nullptr && g_matrixNative)
            {
                void *native = nullptr;
                if (NativePointer(g_camera, native))
                {
                    float value[16]{};
                    bool faulted = false;
                    void *arguments[2] = { &native, value };
                    g_resolver->Invoke(injected, nullptr, arguments, &faulted);
                    if (faulted)
                    {
                        g_matrixNative = false;
                        Trace("matrix: injected off");
                    }
                    else
                    {
                        std::memcpy(out, value, sizeof(value));
                        return true;
                    }
                }
            }
            (void)managed;
            return false;
        }

        // One-shot check that the injected matrix really is the same matrix the
        // managed getter returns. Twice per process, never per frame.
        //
        // OFF BY DEFAULT. The boxed getter allocates a Matrix4x4, and an
        // allocation on a thread that il2cpp did not create takes the GC lock
        // while the game's main thread may be holding it inside Present — the
        // render thread then spins forever with no fault, which is exactly the
        // freeze seen three times (12:48:34 pid 27520, 13:00 pid 13544, both
        // landing between "stage: matrices done" and "stage: camera+matrices
        // done", and both a couple of minutes after inject on a scene reload —
        // ResetCache() clears g_matrixProbed, so the probe re-armed itself).
        // The `_Injected` variant writes into our own buffer and allocates
        // nothing; that path needs no proof to be safe, only to be correct,
        // and it was checked by hand in `matrix: injected` on 2026-11-17.
        void ProbeMatrices(float *view, float *projection)
        {
            g_matrixProbed = true;
            if (m_getViewMatrixNative == nullptr || m_getProjectionNative == nullptr)
            {
                g_matrixNative = false;
                Trace("matrix: injected absent");
                return;
            }
            // Off unless the operator asks for it with an empty
            // %TEMP%\OmegaCore_matrixprobe. Read here directly instead of through
            // SentinelPresent() so this stays independent of declaration order.
            MarkStage("stage: probe sentinel");
            {
                wchar_t marker[MAX_PATH]{};
                if (GetTempPathW(MAX_PATH, marker) != 0)
                {
                    lstrcatW(marker, L"OmegaCore_matrixprobe");
                    if (GetFileAttributesW(marker) == INVALID_FILE_ATTRIBUTES)
                    {
                        g_matrixNative = true;
                        MarkStage("stage: probe off");
                        Trace("matrix: injected (probe off by default)");
                        return;
                    }
                }
            }
            MarkStage("stage: probe managed");
            float check[16]{};
            bool agree = true;
            for (int which = 0; which < 2 && agree; ++which)
            {
                void *injected =
                    which == 0 ? m_getViewMatrixNative : m_getProjectionNative;
                void *managed = which == 0 ? m_getViewMatrix : m_getProjection;
                void *native = nullptr;
                if (!NativePointer(g_camera, native))
                {
                    agree = false;
                    break;
                }
                float value[16]{};
                bool faulted = false;
                void *arguments[2] = { &native, value };
                g_resolver->Invoke(injected, nullptr, arguments, &faulted);
                void *boxed = Invoke1(managed, g_camera, nullptr);
                if (faulted || boxed == nullptr || !g_resolver->ReadBlock(boxed, check, sizeof(check)))
                {
                    agree = false;
                    break;
                }
                const float *source = which == 0 ? view : projection;
                for (int i = 0; i < 16; ++i)
                {
                    const float delta = std::fabs(value[i] - check[i]);
                    if (delta > 0.0001f || std::fabs(value[i] - source[i]) > 0.0001f)
                    {
                        agree = false;
                        break;
                    }
                }
            }
            g_matrixNative = agree;
            Trace(agree ? "matrix: injected" : "matrix: injected mismatch");
        }

        // Rebuilds the camera's Transform ancestor list once per scan. The own
        // body is the character whose Transform chain meets the camera's; the
        // top-most node (the scene root, shared by every character) is dropped,
        // because intersecting on it would flag the whole lobby as "self".
        void BuildCameraChain()
        {
            g_cameraChain.clear();
            g_cameraChainValid = false;
            if (!g_options.selfFilter)
            {
                return;
            }
            RefreshCamera();
            if (g_camera == nullptr || m_getTransform == nullptr || m_getParent == nullptr)
            {
                return;
            }
            void *node = Invoke1(m_getTransform, g_camera, nullptr);
            if (node == nullptr)
            {
                return;
            }
            void *parent = nullptr;
            for (int depth = 0; depth < 16; ++depth)
            {
                g_cameraChain.push_back(node);
                parent = Invoke1(m_getParent, node, nullptr);
                if (parent == nullptr)
                {
                    // `node` is the scene root: every character shares it.
                    if (!g_cameraChain.empty())
                    {
                        g_cameraChain.pop_back();
                    }
                    break;
                }
                node = parent;
            }
            g_cameraChainValid = !g_cameraChain.empty();
        }

        // True when this transform and the camera sit under a common ancestor
        // that is not the scene root.
        bool SelfByChain(void *transform)
        {
            if (!g_cameraChainValid || transform == nullptr || m_getParent == nullptr)
            {
                return false;
            }
            if (g_cameraChain.size() == 1 && g_cameraChain[0] == transform)
            {
                return true;
            }
            void *node = transform;
            for (int depth = 0; depth < 16 && node != nullptr; ++depth)
            {
                for (void *ancestor : g_cameraChain)
                {
                    if (ancestor == node)
                    {
                        return true;
                    }
                }
                node = Invoke1(m_getParent, node, nullptr);
            }
            return false;
        }


        // 128 bytes copied, instead of two invokes and two boxed matrices per
        // entity per frame.
        bool RefreshMatrices()
        {
            if (g_camera == nullptr)
            {
                return false;
            }
            float view[16]{};
            float projection[16]{};
            MarkStage("stage: view matrix");
            if (!ReadMatrix(m_getViewMatrixNative, m_getViewMatrix, view))
            {
                // The previous matrices stay valid for a moment: blanking a frame
                // because one camera read missed is the visible "blink" report.
                return false;
            }
            MarkStage("stage: projection matrix");
            if (!ReadMatrix(m_getProjectionNative, m_getProjection, projection))
            {
                return false;
            }
            MarkStage("stage: matrices done");
            if (!g_matrixProbed && g_matrixNative)
            {
                // The whole stall window between "matrices done" and
                // "camera+matrices done" is labelled statement by statement:
                // every freeze so far stopped in this window, and the last
                // label is the only witness the stuck thread leaves behind.
                MarkStage("stage: probe enter");
                // Proven against what was just read, not against a second read:
                // the two getters are the same value inside one sample.
                ProbeMatrices(view, projection);
                MarkStage("stage: probe done");
            }
            Multiply(projection, view, g_mvp);
            MarkStage("stage: mvp done");
            g_mvpValid = true;
            g_mvpTick = GetTickCount64();

            // Camera world position straight out of the view matrix. The matrix
            // is column-major (Project reads row 0 as m[0], m[4], m[8]), and the
            // view matrix is [A | t] with A = world->camera rotation. The camera
            // sits at the origin of camera space, so t = -A*c and c = -A^T*t.
            // The old code multiplied A by t instead of A^T by t, which put the
            // eye tens of metres away: on pid 24832 "near=" (nearest body to the
            // eye) read 92 m while the farthest visible body was 19.6 m, which
            // cannot both be true. Every geometric self test reads this point.
            const float tx = view[12];
            const float ty = view[13];
            const float tz = view[14];
            g_cameraPosition.x = -(view[0] * tx + view[1] * ty + view[2] * tz);
            g_cameraPosition.y = -(view[4] * tx + view[5] * ty + view[6] * tz);
            g_cameraPosition.z = -(view[8] * tx + view[9] * ty + view[10] * tz);
            g_cameraPositionValid = true;
            {
                // Yaw for the radar: the camera looks down -Z in view space, so
                // the world forward is minus the third row of the rotation. 0
                // degrees faces +Z, growing towards +X.
                const float fx = -view[2];
                const float fz = -view[10];
                g_cameraYawDeg = std::atan2(fx, fz) * (180.0f / 3.14159265358979323846f);
                g_cameraYawValid = (fx != 0.0f || fz != 0.0f);
            }
            {
                // One-shot proof, no guessing: the same point read from the
                // camera's own Transform. A wrong formula here silently kills
                // every distance-based self test, so it is measured once.
                static bool cameraProven = false;
                if (!cameraProven && m_getTransform != nullptr && g_camera != nullptr)
                {
                    void *cameraTransform = Invoke1(m_getTransform, g_camera, nullptr);
                    Vec3 truth{};
                    if (cameraTransform != nullptr && TransformPosition(cameraTransform, truth))
                    {
                        cameraProven = true;
                        char line[192];
                        std::snprintf(line, sizeof(line),
                                      "camera: matrix=%.2f,%.2f,%.2f transform=%.2f,%.2f,%.2f "
                                      "delta=%.2f m",
                                      g_cameraPosition.x, g_cameraPosition.y, g_cameraPosition.z,
                                      truth.x, truth.y, truth.z,
                                      aim::Length(aim::Sub(truth, g_cameraPosition)));
                        Trace(line);
                    }
                }
            }
            MarkStage("stage: refresh done");
            return true;
        }

        // World -> ImGui pixels, entirely in native code. Unity's projection
        // matrix is the gl-style one (y up, clip.w = view-space depth), so the
        // only conversion is ndc -> pixels plus the top-left y flip ImGui wants.
        // The matrix is passed in: the render thread projects with its own
        // snapshot (g_renderMvp) so a camera write on the worker thread can
        // never tear the matrix mid-frame.
        bool ProjectWithMvp(const float *m, bool mvpValid, const Vec3 &world,
                            float displayWidth, float displayHeight, float &sx, float &sy,
                            float &depth)
        {
            if (!mvpValid || m == nullptr)
            {
                return false;
            }
            const float cx = m[0] * world.x + m[4] * world.y + m[8] * world.z + m[12];
            const float cy = m[1] * world.x + m[5] * world.y + m[9] * world.z + m[13];
            const float cw = m[3] * world.x + m[7] * world.y + m[11] * world.z + m[15];
            // Near and far planes: closer than the near clip the projection
            // explodes across the screen, and past the far plane a character is
            // not rendered by the game either, so no box is the honest answer.
            if (!(cw > kNearPlane) || cw > kFarPlane)
            {
                return false;
            }
            const float ndcX = cx / cw;
            const float ndcY = cy / cw;
            depth = cw;
            sx = (ndcX * 0.5f + 0.5f) * displayWidth;
            sy = (1.0f - (ndcY * 0.5f + 0.5f)) * displayHeight;
            return std::isfinite(sx) && std::isfinite(sy);
        }

        bool Project(const Vec3 &world, float displayWidth, float displayHeight, float &sx,
                     float &sy, float &depth)
        {
            return ProjectWithMvp(g_mvp, g_mvpValid, world, displayWidth, displayHeight, sx, sy,
                                  depth);
        }

        // Re-projects the cached positions against the camera as it is right
        // now. This is what makes the esp track the view instead of lagging a
        // sample behind: positions stay on their own timer (one position read
        // per entity is the expensive part), the projection is free and runs
        // every frame.
        void MatchScreenPlates(std::vector<Entity> &entities, bool useRenderPlates,
                               float displayHeight);
        void Reproject(float displayWidth, float displayHeight, bool allowManaged = true,
                       std::vector<Entity> *target = nullptr)
        {
            if (displayWidth <= 1.0f || displayHeight <= 1.0f)
            {
                return;
            }
            std::vector<Entity> &entities = target != nullptr ? *target : g_entities;
            // Which matrix this call may use. The render thread gets its own
            // snapshot of the last published MVP; the worker keeps writing
            // g_mvp in RefreshMatrices(). Reading the shared one from the
            // render thread tore the matrix on a camera turn (half old, half
            // new) and the skeleton jumped for one frame.
            const bool renderThreadView = !allowManaged && g_workerMode;
            const float *activeMvp = renderThreadView ? g_renderMvp : g_mvp;
            const bool activeMvpValid = renderThreadView ? g_renderMvpValid : g_mvpValid;
            // Two invokes per frame total, not per entity: without a fresh
            // camera matrix the boxes would sit where the camera used to point.
            // The camera matrix read is redone every frame; when it fails for one
            // frame (a camera swap, a scene tick) the last good matrices are
            // still a better answer than blanking every box for that frame —
            // that is a visible blink, not a correct "no camera" state.
            MarkStage("stage: reproject enter");
            const bool fresh = allowManaged && RefreshCamera() && RefreshMatrices();
            MarkStage("stage: camera+matrices done");
            if (!fresh &&
                (!activeMvpValid || (!g_workerMode && GetTickCount64() - g_mvpTick > 500)))
            {
                for (Entity &entity : entities)
                {
                    entity.visible = false;
                }
                return;
            }

            const ULONGLONG nowTick = GetTickCount64();
            // Same clock the bone captures are stamped with: the pose phase is
            // the age of the newest read divided by the interval between the two
            // reads it is blended against, and mixing a 1 ms clock with a 15.6 ms
            // one would quantise that quotient into three values.
            const ULONGLONG nowUs = NowMicroseconds();
            // Frame delta for the smoothing filter from the renderer, not from
            // GetTickCount64. That counter steps by 15.6 ms, so a 144 Hz frame
            // reads as 0, 0, 16, 16, 16, 31 and the filter coefficient pulsates
            // between 0.59 and 0.82 on its own - a jitter with no cause in the
            // world. DeltaTime is the interval of the frame being drawn.
            // DeltaTime is the interval of the frame being drawn, clamped so one
            // long frame cannot switch the filters off: at 0.1 s the coefficient
            // of both follows is 1.0, so a stall in a crowd disabled the smoothing
            // exactly when the pose jumped most.
            const float frameSeconds = std::clamp(ImGui::GetIO().DeltaTime, 0.001f, 0.033f);
            const float smoothAlpha = 1.0f - std::exp(-frameSeconds * 1000.0f / kSmoothTauMs);
            for (Entity &entity : entities)
            {
                // The capture (camera matrices + every body's feet) now runs
                // BEFORE this projection, in the same frame. entity.feet is
                // therefore this frame's read, and the old
                // `feet += velocity * sampleAge` was a second prediction on top
                // of a fresh value: it put the box ahead of the body, and right
                // after a Scan() the age reached 40-60 ms, which is when the
                // skeleton flew away. The age is kept for the live: trace only.
                const ULONGLONG sampleAgeUs =
                    (nowUs > entity.sampleTick) ? nowUs - entity.sampleTick : 0;
                Vec3 feet = entity.feet;
                // Low-pass the world anchor: the sample is a 30 Hz staircase and
                // extrapolation alone turns every velocity hiccup into a visible
                // lurch. Anything faster than a walk skips the filter outright -
                // a low-pass on a fast target is pure phase lag, and two of them
                // in series (velocity + anchor) is what made a sprinting player
                // trail the box by 40-60 ms.
                // Only the owning filter store may decide whether the filter
                // restarts. Entity is rebuilt from scratch by every Sample()
                // (125 Hz against a 144 Hz frame), so `entity.smoothInit` was
                // false almost every frame and the reset branch ran instead of
                // the filter: sm= stayed byte-identical to raw= in 88 % of
                // 36599 live: lines even after the state moved into History.
                FilterState *filter = nullptr;
                if (renderThreadView)
                {
                    filter = FindOrCreateRenderFilter(entity.key);
                }
                else
                {
                    History *history = FindHistory(entity.key);
                    if (history != nullptr)
                    {
                        filter = &history->filter;
                    }
                }
                if (filter == nullptr || !filter->smoothInit ||
                    nowTick - filter->smoothTick > 400 ||
                    aim::Length(entity.velocity) > kSmoothBypassSpeed ||
                    aim::Length(aim::Sub(feet, filter->smoothFeet)) > 3.0f)
                {
                    entity.smoothFeet = feet;
                    entity.smoothInit = true;
                    if (filter != nullptr)
                    {
                        filter->smoothFeet = feet;
                        filter->smoothInit = true;
                        // A teleport or a long gap invalidates the joint follow
                        // too: the captured pose belongs to a different place.
                        filter->bonesSmoothValid = false;
                        ResetBoneHistory(*filter);
                    }
                }
                else
                {
                    // No velocity feed-forward. It existed to cancel the lag of
                    // a filter fed by a stale sample; with the capture in this
                    // frame the filter only has to polish read noise, and
                    // advancing the state by velocity * frameSeconds put the
                    // predicted point in front of the body every single frame.
                    filter->smoothFeet.x += (feet.x - filter->smoothFeet.x) * smoothAlpha;
                    filter->smoothFeet.y += (feet.y - filter->smoothFeet.y) * smoothAlpha;
                    filter->smoothFeet.z += (feet.z - filter->smoothFeet.z) * smoothAlpha;
                    entity.smoothFeet = filter->smoothFeet;
                    entity.smoothInit = true;
                }
                if (filter != nullptr)
                {
                    filter->smoothTick = nowTick;
                }
                entity.smoothTick = nowTick;
                feet = entity.smoothFeet;

                float depth = 0.0f;
                entity.visible = ProjectWithMvp(activeMvp, activeMvpValid, feet, displayWidth,
                                                displayHeight, entity.feetX, entity.feetY, depth);
                entity.distance = depth;
                // Same 240 px margin the bone pass culls with: this is the set
                // of bodies whose joints are supposed to be fresh, so boneAge
                // has to be measured over it and nothing else.
                constexpr float kAgeMarginPixels = 240.0f;
                entity.onScreen = entity.visible &&
                                  entity.feetX >= -kAgeMarginPixels &&
                                  entity.feetX <= displayWidth + kAgeMarginPixels &&
                                  entity.feetY >= -kAgeMarginPixels &&
                                  entity.feetY <= displayHeight + kAgeMarginPixels;
                // One line per second for anything moving faster than a walk:
                // with the extrapolation gone, a box that still runs in front of
                // the body means the position we read is already ahead of the
                // rendered character — a different bug from a bad prediction,
                // and this line is the only way to tell the two apart without
                // guessing.
                //
                // Off unless %TEMP%\OmegaCore_livetrace exists. traceTick used
                // to live on the Entity, which Sample() rebuilds, so a body
                // whose bone pass is skipped (behind the camera, d=0.0) reset it
                // to zero every frame and the 1 s throttle never engaged: the
                // trace then opened, wrote and closed a file per frame on the
                // Present thread.
                if (nowTick - entity.traceTick > 1000 && LiveTraceEnabled() &&
                    aim::Length(entity.velocity) > 3.0f)
                {
                    entity.traceTick = nowTick;
                    char live[256];
                    std::snprintf(live, sizeof(live),
                                  "live: raw=%.2f,%.2f,%.2f v=%.2f,%.2f spd=%.1f "
                                  "sm=%.2f,%.2f,%.2f d=%.1f age=%llu",
                                  entity.feet.x, entity.feet.y, entity.feet.z,
                                  entity.velocity.x, entity.velocity.z,
                                  aim::Length(entity.velocity),
                                  entity.smoothFeet.x, entity.smoothFeet.y,
                                  entity.smoothFeet.z, entity.distance,
                                  static_cast<unsigned long long>(sampleAgeUs / 1000ull));
                    TraceLine(live);
                }

                // The pose is read a couple of characters per tick, so between
                // reads the figure has to be carried forward. Joints are stored
                // as offsets from the anchor, and this frame's feet already carry
                // the sample-age extrapolation, so the whole rig lands on the
                // live body: one rigid translation, no per-joint velocity, no
                // "live feet - pose anchor" clip that snapped a car-mounted rig
                // back and forth when the pose went stale.
                const float baseX = feet.x + entity.anchorOffset.x;
                const float baseY = feet.y + entity.anchorOffset.y;
                const float baseZ = feet.z + entity.anchorOffset.z;

                // No pose look-ahead. It assumed the joints move along the step
                // between two captures, and between captures the offsets only
                // change when the next capture lands. In a crowd (8-17 bodies,
                // BonePass budget is shared) the captures are far apart, so the
                // drawn joint sat at capture+phase and then jumped back to the
                // new capture with phase 0: a sawtooth of 0.15 of a step on every
                // joint, twice per capture interval. On a firing character the
                // step is recoil and strafe, so the sawtooth read as hard shaking
                // of the arms. Rigid translation between captures is already
                // covered by the extrapolated feet below; only the pose delta is
                // smoothed, by the follow filter.
                // Per-entity follow filter. A body that just appeared, was not
                // drawn for a while or teleported starts on the captured pose
                // instead of sliding in from wherever it was last seen. The
                // state lives in History for the same reason the anchor filter
                // does: Sample() rebuilds Entity and the old inheritance was
                // tied to a non-zero boneTick.
                const bool boneStale = filter == nullptr || !filter->bonesSmoothValid ||
                                       nowTick - filter->bonesSmoothTick > 400;
                // A capture lands every 8-30 ms, Reproject runs every frame.
                // Counting jump frames on a 144 Hz clock "confirmed" a spike in
                // 14 ms, so a pooled joint that belonged to a neighbour for one
                // capture was accepted almost instantly. The streak counts
                // CAPTURES: a joint has to be seen in the same wrong place on
                // two separate reads before the new position is believed.
                const bool newCapture = entity.boneTick != 0 && filter != nullptr &&
                                        entity.boneTick != filter->lastBoneTick;
                bool anyBoneVisible = false;

                // Seating and unseating move the whole figure at once. Counting
                // how many of this rig's joints stepped further than the snap
                // threshold tells a body-wide pose change from a single bad
                // joint, and a body-wide change must be taken immediately -
                // otherwise the pose hangs in the air until the streak fills.
                // Only the core is counted: arms are handled as one limb in
                // BonePass (a pre-IK hand step is 0.9 m and would make every
                // armed body look like it teleported), so a body-wide accept
                // must not be triggered by the four arm joints.
                int bigStepBones = 0;
                if (!boneStale && newCapture && filter != nullptr && filter->bonesSmoothValid)
                {
                    for (int b = 0; b <= BoneHead; ++b)
                    {
                        if (!entity.boneVisible[b])
                        {
                            continue;
                        }
                        const Vec3 d = aim::Sub(entity.bones[b], filter->bonesSmooth[b]);
                        if (d.x * d.x + d.y * d.y + d.z * d.z > kBoneSnapStep * kBoneSnapStep)
                        {
                            ++bigStepBones;
                        }
                    }
                }
                const bool wholeBodySnap = (bigStepBones >= 4);
                // Dense anatomical limit measured from the body's own hips, so a
                // joint borrowed from a neighbour is still rejected. When no
                // torso joint read this frame the check is SKIPPED rather than
                // centred on a standing height: `feet + 0.95` put the centre
                // 0.95 m above the ground on a body seated in a car, whose real
                // torso sits at that spot, so every joint of a seated player was
                // rejected by the tight box - the skeleton vanishing on seating.
                bool haveHipsOffset = false;
                Vec3 hipsOffset{};
                if (entity.boneVisible[BoneHips])
                {
                    hipsOffset = entity.bones[BoneHips];
                    haveHipsOffset = true;
                }
                else if (entity.boneVisible[BoneSpine])
                {
                    hipsOffset = aim::Sub(entity.bones[BoneSpine], Vec3{0.0f, 0.30f, 0.0f});
                    haveHipsOffset = true;
                }
                else if (entity.boneVisible[BoneChest])
                {
                    hipsOffset = aim::Sub(entity.bones[BoneChest], Vec3{0.0f, 0.55f, 0.0f});
                    haveHipsOffset = true;
                }

                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    // No screen-space cache. Holding a joint's old pixels pins
                    // it while the camera turns and the rest of the body turns
                    // with the camera, then it snaps back when the cache
                    // expires - a stutter on every camera move. A joint that
                    // did not project this frame is simply not drawn.
                    if (!entity.boneVisible[bone])
                    {
                        entity.boneOnScreen[bone] = false;
                        continue;
                    }
                    anyBoneVisible = true;
                    // The measured pose, whole. The follow filter below is the
                    // only thing between the capture and the screen.
                    const Vec3 captured = entity.bones[bone];
                    {
                        // Two limits, because one cannot do both jobs. The wide
                        // one from poseAnchor covers the model shifting into a
                        // seat; the tight one from the hips still rejects a
                        // joint that belongs to the body standing next to it.
                        const float rootHorizSq = captured.x * captured.x + captured.z * captured.z;
                        if (rootHorizSq > 3.5f * 3.5f || captured.y < -2.0f || captured.y > 2.8f)
                        {
                            entity.boneOnScreen[bone] = false;
                            continue;
                        }
                        if (haveHipsOffset)
                        {
                            const Vec3 fromHips = aim::Sub(captured, hipsOffset);
                            const float hipsHorizSq =
                                fromHips.x * fromHips.x + fromHips.z * fromHips.z;
                            if (hipsHorizSq > 1.35f * 1.35f || fromHips.y < -1.35f ||
                                fromHips.y > 1.35f)
                            {
                                entity.boneOnScreen[bone] = false;
                                continue;
                            }
                        }
                    }
                    if (boneStale && filter != nullptr)
                    {
                        // First capture after a teleport, a scene change or a
                        // 400 ms gap: seed the ring so the medoid votes on real
                        // data instead of the empty history.
                        filter->boneHist[bone][0] = captured;
                        filter->boneHistCount[bone] = 1;
                        filter->boneHistPos[bone] = 1;
                    }
                    Vec3 offset = captured;
                    if (!boneStale && filter != nullptr && filter->bonesSmoothValid)
                    {
                        if (newCapture)
                        {
                            const int slotIndex = filter->boneHistPos[bone] % kBoneHistCap;
                            filter->boneHist[bone][slotIndex] = captured;
                            filter->boneHistPos[bone] =
                                static_cast<unsigned char>((slotIndex + 1) % kBoneHistCap);
                            if (filter->boneHistCount[bone] < kBoneHistCap)
                            {
                                ++filter->boneHistCount[bone];
                            }
                        }
                        // Every joint votes through the medoid, not just the
                        // arms: the head and torso shake at the bar is the same
                        // read race with a smaller amplitude, and the old
                        // median-of-three covered only the four arm joints.
                        // Seven captures keep the post-IK point through a burst
                        // of three pre-IK reads; a running body drops to three
                        // so the filter adds no lag to a step.
                        Vec3 target = captured;
                        const int avail = static_cast<int>(filter->boneHistCount[bone]);
                        // A moving body skips the medoid entirely. Three stale
                        // captures are a freeze followed by a jump when the body
                        // covers 0.3-0.9 m between them; only a standing target
                        // needs the vote against the pre-IK read race.
                        const int wantWin = (entity.speed < 0.8f) ? kBoneHistCap : 0;
                        const int win = std::min(avail, wantWin);
                        if (win >= 3)
                        {
                            Vec3 recent[kBoneHistCap];
                            for (int i = 0; i < win; ++i)
                            {
                                const int idx =
                                    (static_cast<int>(filter->boneHistPos[bone]) - 1 - i +
                                     kBoneHistCap) % kBoneHistCap;
                                recent[i] = filter->boneHist[bone][idx];
                            }
                            target = SpatialMedoid(recent, win);
                            // If every capture in the window sits within one
                            // agreement distance of the vote, the window holds
                            // one place: no weapon race, so the vote is only
                            // adding the age of the middle capture as lag. On a
                            // standing body aiming or reloading that is a limb
                            // 50-100 ms behind the model - the "bones lead or
                            // trail" report. A race spreads the window (one
                            // capture 0.3-0.9 m away), so the wide case keeps
                            // the medoid untouched.
                            float spread = 0.0f;
                            for (int i = 0; i < win; ++i)
                            {
                                spread = std::max(spread, aim::Length(aim::Sub(recent[i], target)));
                            }
                            if (spread <= kJumpAgreeMeters)
                            {
                                target = captured;
                            }
                        }
                        if (wholeBodySnap)
                        {
                            // The whole figure moved at once (seated, unseated,
                            // knocked down). Take it now and drop the votes, so
                            // the next capture starts clean.
                            filter->boneJumpStreak[bone] = 0;
                            filter->bonePendingValid[bone] = false;
                            filter->boneHistCount[bone] = 0;
                            filter->boneHistPos[bone] = 0;
                            offset = captured;
                        }
                        else
                        {
                            const Vec3 &current = filter->bonesSmooth[bone];
                            const float dx = target.x - current.x;
                            const float dy = target.y - current.y;
                            const float dz = target.z - current.z;
                            const float stepSq = dx * dx + dy * dy + dz * dz;
                            const bool bodyStill = (entity.speed < 0.35f);
                            if (bodyStill && stepSq < 0.02f * 0.02f)
                            {
                                // Idle micro-shake of the head, neck and torso
                                // is under 2 cm: kill it instead of filtering.
                                filter->boneJumpStreak[bone] = 0;
                                filter->bonePendingValid[bone] = false;
                                offset = current;
                            }
                            else
                            {
                                // A still body glides through steps under 18 cm
                                // (leftovers of LookAt and weapon IK) at 28 ms;
                                // real motion keeps the 8 ms follow. 65 ms was
                                // picked against the weapon race alone and read
                                // as the hand trailing the model by a beat.
                                const float tauMs =
                                    (bodyStill && stepSq < 0.18f * 0.18f) ? 28.0f
                                                                           : kBoneSmoothTauMs;
                                const float alpha =
                                    1.0f - std::exp(-frameSeconds * 1000.0f / tauMs);
                                // A standing body needs three agreeing captures
                                // (first hold, then two confirmations) before a
                                // step over 30 cm is believed; a moving one does
                                // not have the time for that.
                                // A moving body does not wait for two agreeing
                                // captures: at sprint speed the joint genuinely
                                // is that far on the next read, and holding it
                                // for another capture IS the freeze-then-catch-up
                                // the report describes. The gate stays for a
                                // standing target, where the weapon race lives.
                                const int neededStreak = bodyStill ? 2 : 1;
                                const bool bigJump =
                                    bodyStill && stepSq > kBoneSnapStep * kBoneSnapStep;
                                if (!bigJump)
                                {
                                    filter->boneJumpStreak[bone] = 0;
                                    filter->bonePendingValid[bone] = false;
                                    offset = Vec3{ current.x + dx * alpha,
                                                   current.y + dy * alpha,
                                                   current.z + dz * alpha };
                                }
                                else if (!newCapture)
                                {
                                    offset = current;
                                }
                                else if (filter->boneJumpStreak[bone] < neededStreak)
                                {
                                    if (filter->boneJumpStreak[bone] == 0 ||
                                        !filter->bonePendingValid[bone] ||
                                        aim::Length(aim::Sub(target, filter->bonePending[bone])) <=
                                            kJumpAgreeMeters)
                                    {
                                        ++filter->boneJumpStreak[bone];
                                        filter->bonePending[bone] = target;
                                        filter->bonePendingValid[bone] = true;
                                    }
                                    else
                                    {
                                        filter->boneJumpStreak[bone] = 1;
                                        filter->bonePending[bone] = target;
                                        filter->bonePendingValid[bone] = true;
                                    }
                                    offset = current;
                                }
                                else
                                {
                                    const bool agreed =
                                        !filter->bonePendingValid[bone] ||
                                        aim::Length(aim::Sub(target, filter->bonePending[bone])) <=
                                            kJumpAgreeMeters;
                                    if (!agreed)
                                    {
                                        filter->boneJumpStreak[bone] = 1;
                                        filter->bonePending[bone] = target;
                                        filter->bonePendingValid[bone] = true;
                                        offset = current;
                                    }
                                    else
                                    {
                                        filter->boneJumpStreak[bone] = 0;
                                        filter->bonePendingValid[bone] = false;
                                        offset = target;
                                    }
                                }
                            }
                        }
                    }
                    // Drawn arm step, measured against the value this pass is
                    // about to replace. Reported by walk: so the medoid and the
                    // hold are judged by what reaches the screen, not by the raw
                    // capture BonePass complains about.
                    if (bone >= BoneLeftShoulder && filter != nullptr)
                    {
                        const Vec3 &was = filter->bonesSmooth[bone];
                        const float sx = offset.x - was.x;
                        const float sy = offset.y - was.y;
                        const float sz = offset.z - was.z;
                        const float drawn = std::sqrt(sx * sx + sy * sy + sz * sz);
                        if (drawn > g_drawnArmJumpM)
                        {
                            g_drawnArmJumpM = drawn;
                        }
                    }
                    entity.bonesSmooth[bone] = offset;
                    if (filter != nullptr)
                    {
                        filter->bonesSmooth[bone] = offset;
                    }
                    Vec3 joint = Vec3{ baseX + offset.x, baseY + offset.y, baseZ + offset.z };
                    float depth = 0.0f;
                    float screenX = 0.0f;
                    float screenY = 0.0f;
                    const bool onScreen =
                        ProjectWithMvp(activeMvp, activeMvpValid, joint, displayWidth,
                                       displayHeight, screenX, screenY, depth);
                    if (onScreen)
                    {
                        entity.boneX[bone] = screenX;
                        entity.boneY[bone] = screenY;
                    }
                    entity.boneOnScreen[bone] = onScreen;
                }
                // A driver whose feet are behind the near plane still has a head
                // and chest inside the window. Dropping the whole entity because
                // the feet did not project is what made the ESP vanish right
                // next to a car.
                if (!entity.visible && anyBoneVisible)
                {
                    const int anchors[2] = { BoneHead, BoneChest };
                    for (int a = 0; a < 2; ++a)
                    {
                        const int bone = anchors[a];
                        if (!entity.boneOnScreen[bone])
                        {
                            continue;
                        }
                        const Vec3 point = Vec3{ baseX + entity.bonesSmooth[bone].x,
                                                 baseY + entity.bonesSmooth[bone].y,
                                                 baseZ + entity.bonesSmooth[bone].z };
                        float anchorX = 0.0f;
                        float anchorY = 0.0f;
                        float anchorDepth = 0.0f;
                        if (ProjectWithMvp(activeMvp, activeMvpValid, point, displayWidth,
                                           displayHeight, anchorX, anchorY, anchorDepth))
                        {
                            entity.visible = true;
                            entity.distance = anchorDepth;
                            entity.feetX = anchorX;
                            entity.feetY = displayHeight;
                            break;
                        }
                    }
                }

                if (anyBoneVisible)
                {
                    entity.bonesSmoothValid = true;
                    entity.bonesSmoothTick = nowTick;
                    if (filter != nullptr)
                    {
                        filter->bonesSmoothValid = true;
                        filter->bonesSmoothTick = nowTick;
                        filter->lastBoneTick = entity.boneTick;
                    }
                }

                // The head the box top and the labels hang off is the DRAWN
                // joint, not the raw capture: a raw head moves on the capture
                // clock while the rest of the figure follows the frame clock,
                // and the nickname jumps with it.
                float headX = 0.0f;
                float headY = 0.0f;
                float headDepth = 0.0f;
                Vec3 head = Vec3{ entity.smoothFeet.x + (entity.head.x - entity.feet.x),
                                  entity.smoothFeet.y + (entity.head.y - entity.feet.y),
                                  entity.smoothFeet.z + (entity.head.z - entity.feet.z) };
                if (entity.boneVisible[BoneHead] && entity.bonesSmoothValid)
                {
                    head = Vec3{ baseX + entity.bonesSmooth[BoneHead].x,
                                 baseY + entity.bonesSmooth[BoneHead].y,
                                 baseZ + entity.bonesSmooth[BoneHead].z };
                }
                if (entity.visible &&
                    ProjectWithMvp(activeMvp, activeMvpValid, head, displayWidth, displayHeight,
                                   headX, headY, headDepth))
                {
                    entity.headX = headX;
                    entity.headY = headY;
                }
                else
                {
                    // Falls back to an estimated box height so a partially
                    // off-screen character still gets a sane rectangle.
                    entity.headX = entity.feetX;
                    entity.headY = entity.feetY - 10.0f;
                }

                // Lead point in screen space: project the predicted world head
                // and keep the offset from the measured one. A jump bigger than
                // the cap is a torn sample, not a sprint, so it is cut back.
                float leadX = 0.0f;
                float leadY = 0.0f;
                float predictedX = 0.0f;
                float predictedY = 0.0f;
                float predictedDepth = 0.0f;
                if (entity.visible && ProjectWithMvp(activeMvp, activeMvpValid,
                                                     entity.predictedHead, displayWidth,
                                                     displayHeight, predictedX, predictedY,
                                                     predictedDepth))
                {
                    leadX = predictedX - entity.headX;
                    leadY = predictedY - entity.headY;
                    const float magnitude = std::sqrt(leadX * leadX + leadY * leadY);
                    if (!aim::Finite(magnitude))
                    {
                        leadX = 0.0f;
                        leadY = 0.0f;
                    }
                    else if (magnitude > kMaxLeadPixels)
                    {
                        const float scale = kMaxLeadPixels / magnitude;
                        leadX *= scale;
                        leadY *= scale;
                    }
                }
                entity.leadX = leadX;
                entity.leadY = leadY;
                entity.predictedHeadX = entity.headX + leadX;
                entity.predictedHeadY = entity.headY + leadY;

                // Box quad straight from the projected joints: the head is the
                // top, the widest joint sets the sides, the lowest point sits on
                // the ground. Measured, not estimated from a height ratio, and
                // never fed by the aim lead — the prediction is an aim aid, and
                // folding it into the rectangle is what made the box float up
                // and look uneven. There is no per-frame easing pass: the joints
                // already ride the extrapolated feet, and a constant blend factor
                // is both a lag (a runner's box trails by a frame and a half) and
                // frame-rate dependent, which reads as judder on a fast-moving
                // target.
                float minY = 0.0f;
                bool anyJoint = false;
                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    if (!entity.boneOnScreen[bone])
                    {
                        continue;
                    }
                    if (!anyJoint)
                    {
                        minY = entity.boneY[bone];
                        anyJoint = true;
                        continue;
                    }
                    minY = std::min(minY, entity.boneY[bone]);
                }

                if (anyJoint)
                {
                    // The body, not the silhouette: the top is the head bone,
                    // the bottom is the lowest foot (or the transform origin,
                    // which sits on the ground), and the width is the shoulder
                    // span. Using the extremes of every joint made the box
                    // balloon the moment a player raised a rifle.
                    // The top is the CROWN joint, not the topmost joint of the
                    // whole body: a raised hand, a rifle or a weapon mount lifted
                    // the rectangle (and the name hanging off its top edge) off
                    // the head, which is what made the label sit too high.
                    const float top = entity.boneOnScreen[BoneHead]
                                          ? std::min(entity.boneY[BoneHead], entity.headY)
                                          : std::min(minY, entity.headY);
                    float bottom = entity.feetY;
                    const int feet[] = { BoneLeftFoot, BoneRightFoot, BoneLeftLowerLeg,
                                         BoneRightLowerLeg };
                    for (int foot : feet)
                    {
                        if (entity.boneOnScreen[foot])
                        {
                            bottom = std::max(bottom, entity.boneY[foot]);
                        }
                    }
                    const float height = std::max(bottom - top, 1.0f);
                    float width = height * 0.30f;
                    // Centre: the shoulder span when both shoulders projected,
                    // otherwise the hips of the SAME rig. Falling back to the
                    // body's feet pixel put the rectangle on the model root
                    // while the joints sat on the rig root, which is the box
                    // drawn beside its own skeleton.
                    float centre = entity.feetX;
                    if (entity.boneOnScreen[BoneHips])
                    {
                        centre = entity.boneX[BoneHips];
                    }
                    if (entity.boneOnScreen[BoneLeftShoulder] &&
                        entity.boneOnScreen[BoneRightShoulder])
                    {
                        width = std::max(std::fabs(entity.boneX[BoneRightShoulder] -
                                                   entity.boneX[BoneLeftShoulder]) *
                                             1.25f,
                                         height * 0.26f);
                        centre = (entity.boneX[BoneLeftShoulder] +
                                  entity.boneX[BoneRightShoulder]) *
                                 0.5f;
                    }
                    entity.boxLeft = centre - width * 0.5f;
                    entity.boxRight = centre + width * 0.5f;
                    entity.boxTop = top;
                    entity.boxBottom = bottom;
                    entity.boxFromBones = true;
                    if (filter != nullptr)
                    {
                        filter->lastWidthRatio = width / height;
                        filter->lastHeadOffsetPx = entity.feetY - top;
                        filter->lastBoxTick = GetTickCount64();
                    }
                }
                else
                {
                    float height = entity.feetY - entity.headY;
                    // A body with no joints has no measured head either: when the
                    // head point misses the projection the code falls back to
                    // `feetY - 10`, and 10 px times 0.28 is the single thin stick
                    // Olivia saw on a seated player. Take the analytic height of a
                    // standing body at this depth instead - row 0 of the MVP is
                    // the screen-X axis already scaled by the focal length, so
                    // `|row0| * 0.5 * width / depth` is pixels per metre.
                    constexpr float kFallbackBodyMetres = 1.80f;
                    constexpr float kMinFallbackHeight = 28.0f;
                    if (height < kMinFallbackHeight && activeMvpValid && entity.distance > 0.25f)
                    {
                        const float row = std::sqrt(activeMvp[0] * activeMvp[0] +
                                                    activeMvp[4] * activeMvp[4] +
                                                    activeMvp[8] * activeMvp[8]);
                        height = std::max(row * 0.5f * displayWidth * kFallbackBodyMetres /
                                              entity.distance,
                                          kMinFallbackHeight);
                    }
                    if (height < 6.0f)
                    {
                        height = 6.0f;
                    }
                    // 0.28, not 0.42. A boneless box used to be 1.5x wider than
                    // the same body measured by its own shoulders (live trace:
                    // 120 px wide without joints, 75 px with them, same player,
                    // same distance). A body keeps its own last ratio for two
                    // seconds, so a rig that drops for a moment - a car door, a
                    // seat, one pooled read - does not visibly inflate.
                    constexpr ULONGLONG kBoxMemoryMs = 2500;
                    const bool remembered =
                        filter != nullptr && filter->lastWidthRatio > 0.05f &&
                        filter->lastWidthRatio < 0.80f &&
                        GetTickCount64() - filter->lastBoxTick <= kBoxMemoryMs;
                    const float width = height * (remembered ? filter->lastWidthRatio : 0.28f);
                    entity.boxLeft = entity.feetX - width * 0.5f;
                    entity.boxRight = entity.feetX + width * 0.5f;
                    if (remembered && filter->lastHeadOffsetPx > 0.0f)
                    {
                        entity.boxTop = entity.feetY - filter->lastHeadOffsetPx;
                    }
                    else
                    {
                        // `headY` is the placeholder feetY - 10 whenever the head
                        // point missed the projection; the analytic height above
                        // is the only usable top in that case.
                        entity.boxTop = std::min(entity.headY, entity.feetY - height);
                    }
                    entity.boxBottom = entity.feetY;
                    entity.boxFromBones = false;
                }
            }

            // Bind screen-space nickname plates to their Entity boxes now that
            // every Entity has valid visible/boxTop from the projection above.
            // useRenderPlates = (target != nullptr): the render thread passes its
            // own g_renderEntities snapshot; the worker uses g_plates directly.
            MatchScreenPlates(entities, target != nullptr, displayHeight);

            // What this frame actually shows, counted where it is known. Sample()
            // rebuilds every Entity with visible=false, so the same count taken
            // there read 0 projected and boneAge=0 ms even with fifteen
            // skeletons on screen.
            size_t projectedCount = 0;
            unsigned long long oldestAgeUs = 0;
            bool haveProbe = false;
            const ULONGLONG ageNowUs = NowMicroseconds();
            for (const Entity &entity : entities)
            {
                if (!entity.visible)
                {
                    continue;
                }
                ++projectedCount;
                if (!haveProbe)
                {
                    haveProbe = true;
                    g_probeFeet = entity.feet;
                    g_probeFeetX = entity.feetX;
                    g_probeFeetY = entity.feetY;
                }
                // Same rule as before: bodies the bone pass deliberately skips
                // are not part of the age of what is drawn.
                if (entity.onScreen && entity.boneVisible[BoneHead] && entity.boneTick != 0 &&
                    ageNowUs > entity.boneTick)
                {
                    oldestAgeUs = std::max(oldestAgeUs, ageNowUs - entity.boneTick);
                }
            }
            g_lastProjected = projectedCount;
            g_lastBoneAgeMs = oldestAgeUs / 1000ull;
            g_probeValid = haveProbe;
        }

        History *FindHistory(void *key)
        {
            for (History &entry : g_history)
            {
                if (entry.key == key)
                {
                    return &entry;
                }
            }
            return nullptr;
        }

        // Render-thread filter store, used only when the projection runs on the
        // render thread (worker mode). It never touches g_history, which the
        // worker thread pushes into and reallocates.
        FilterState *FindOrCreateRenderFilter(void *key)
        {
            if (key == nullptr)
            {
                return nullptr;
            }
            for (RenderFilterState &state : g_renderFilters)
            {
                if (state.key == key)
                {
                    return &state.filter;
                }
            }
            g_renderFilters.push_back(RenderFilterState{});
            g_renderFilters.back().key = key;
            return &g_renderFilters.back().filter;
        }

        // Drops history for components the sample no longer returns. Called once
        // per sample with the live set, so a respawned character starts clean
        // instead of inheriting the velocity of whatever held the slot before.
        // Swap-and-pop: order means nothing here, and erase() in a loop moved
        // the whole tail once per dead entry.
        void PruneHistory()
        {
            for (size_t i = 0; i < g_history.size();)
            {
                bool live = false;
                for (const Entity &entity : g_entities)
                {
                    if (entity.key == g_history[i].key)
                    {
                        live = true;
                        break;
                    }
                }
                if (live)
                {
                    ++i;
                }
                else
                {
                    g_history[i] = g_history.back();
                    g_history.pop_back();
                }
            }
        }

        // Velocity from finite differences between two samples, blended into the
        // running estimate. World units only: the lead is projected every frame
        // from this vector, so no pixel-space velocity has to be tracked.
        // Fastest a body can legitimately move: the quickest vehicle in the
        // game. Anything above this is a teleport, a respawn or a slot reuse,
        // not motion. The old bound was 20000 m/s (72000 km/h), so a respawn
        // produced a "velocity" of 5568 m/s and one frame of extrapolation
        // threw the box and the skeleton off screen.
        constexpr float kMaxTrackedSpeed = 45.0f;

        // The anchor lead is driven by this velocity, so the velocity must not be
        // derived from an 8-16 ms difference: a read race of 3 cm across such a
        // step is 2-4 m/s of noise, and the old code fed that straight into the
        // projection. The delta accumulates for 35 ms, so what gets divided by
        // its own dt is a step worth dividing.
        constexpr ULONGLONG kVelocityAccumulateUs = 35000;

        // Returns false when the sample was rejected as a teleport, so the
        // caller can also drop the smoothing state that rides on it.
        bool UpdateKinematics(History &history, const Vec3 &feet, ULONGLONG nowMicros)
        {
            const ULONGLONG elapsed = nowMicros - history.microTick;
            if (history.microTick != 0 && elapsed < kVelocityAccumulateUs)
            {
                // Too soon: keep the estimate AND the baseline. Moving the
                // baseline here made the next delta 8 ms wide again, which is
                // the exact noise this gate exists to avoid.
                return true;
            }
            if (history.microTick != 0 && elapsed <= 2000000)
            {
                const float dt = static_cast<float>(elapsed) / 1000000.0f;
                const Vec3 sample = aim::Scale(aim::Sub(feet, history.feet), 1.0f / dt);
                if (aim::Finite(sample.x) && aim::Finite(sample.y) && aim::Finite(sample.z) &&
                    aim::Length(sample) > kMaxTrackedSpeed)
                {
                    // Teleport or respawn: forget the old track entirely rather
                    // than carry a nonsense velocity into the next frames.
                    history.velocity = Vec3{};
                    history.feet = feet;
                    history.microTick = nowMicros;
                    return false;
                }
                if (aim::Finite(sample.x) && aim::Finite(sample.y) && aim::Finite(sample.z))
                {
                    // 0.18, not 0.55. With a 35 ms baseline the new sample is
                    // already a real measurement, and a heavy blend is what
                    // keeps a single noisy step from becoming a velocity spike
                    // that the box then follows. 0.55 reached full speed in four
                    // samples at the old 8 ms cadence; at 35 ms that is a slower
                    // ramp anyway, and the anchor no longer needs the velocity
                    // to be current to the frame.
                    history.velocity = aim::Blend(history.velocity, sample, 0.18f);
                    history.feet = feet;
                    history.microTick = nowMicros;
                    return true;
                }
            }

            // First sample, or a delta too small/large to trust: store the
            // position and keep whatever estimate exists until it goes stale.
            if (history.microTick == 0 || elapsed > kVelocityDropMs * 1000ull)
            {
                history.velocity = Vec3{};
            }
            history.feet = feet;
            history.microTick = nowMicros;
            return true;
        }

        // How far ahead of the target the aim point is placed, in seconds.
        float LeadSeconds()
        {
            if (!g_options.aimPrediction)
            {
                return 0.0f;
            }
            return std::max(g_options.aimLeadMs, 0.0f) / 1000.0f;
        }

        void *ReadPointer(void *base, size_t offset);
        bool Alive(void *object);

        // Defined next to the joint matcher, needed here first: an Animator the
        // parent walk found may belong to the vehicle the character is riding
        // in, and the rig walk must not start on a car.
        void NormaliseName(const char *name, char *out, size_t capacity);
        bool HasWord(const char *words, const char *word);

        // The live name of a Transform. Object.get_name works on Transform
        // directly (it is a UnityEngine.Object), which is one managed call.
        bool TransformNodeName(void *transform, char *out, size_t capacity)
        {
            out[0] = '\0';
            if (transform == nullptr || m_getName == nullptr)
            {
                return false;
            }
            void *managed = Invoke1(m_getName, transform, nullptr);
            if (managed == nullptr)
            {
                return false;
            }
            const std::string text = g_resolver->ReadString(managed, 48);
            if (text.empty())
            {
                return false;
            }
            std::snprintf(out, capacity, "%s", text.c_str());
            return true;
        }

        // A node that belongs to a character rather than to the world around it.
        bool LooksLikeCharacterName(const char *raw)
        {
            char words[96];
            NormaliseName(raw, words, sizeof(words));
            return HasWord(words, "player") || HasWord(words, "man") || HasWord(words, "woman") ||
                   HasWord(words, "main rig body") || HasWord(words, "character");
        }

        // A node the rig walk must not climb into or start from.
        bool LooksLikeVehicleName(const char *raw)
        {
            char words[96];
            NormaliseName(raw, words, sizeof(words));
            return HasWord(words, "car") || HasWord(words, "vehicle") || HasWord(words, "seat") ||
                   HasWord(words, "wheel") || HasWord(words, "chassis") || HasWord(words, "trailer") ||
                   HasWord(words, "building");
        }

        // The Animator owns the biped skeleton. It usually sits on the character
        // root: found by walking up from the view, or down when the view itself
        // is the root, so a hierarchy rearrangement costs the skeleton, not the
        // whole walk.
        void *ResolveAnimator(void *component)
        {
            if (component == nullptr || g_animatorType == nullptr)
            {
                return nullptr;
            }
            bool includeInactive = true;
            void *animator = nullptr;
            // Children FIRST. The character's own Animator hangs on a node under
            // the view; a body seated in a car also has the vehicle's Animator
            // above it. The parent walk found that one first, the rig walk then
            // started on the car and found no joints at all — a seated player
            // lost the skeleton for as long as the ride lasted.
            if (m_getComponentInChildren != nullptr)
            {
                animator =
                    Invoke2(m_getComponentInChildren, component, g_animatorType, &includeInactive);
            }
            if (animator != nullptr)
            {
                return animator;
            }
            if (m_getComponentInParent == nullptr)
            {
                return nullptr;
            }
            animator = Invoke2(m_getComponentInParent, component, g_animatorType, &includeInactive);
            if (animator == nullptr)
            {
                return nullptr;
            }
            // Whatever the parent walk found has to belong to a character. A car
            // or a building Animator is not a rig, and starting the name walk
            // from one burns the node budget on wheels and body panels.
            if (m_getTransform != nullptr)
            {
                char name[48] = "";
                void *node = Invoke1(m_getTransform, animator, nullptr);
                if (node != nullptr && TransformNodeName(node, name, sizeof(name)) &&
                    (LooksLikeVehicleName(name) ||
                     !LooksLikeCharacterName(name)))
                {
                    return nullptr;
                }
            }
            return animator;
        }

        // Exact self test. BattleCharacterEntityView keeps the transform the game
        // drives the third-person camera from (FollowCamera 0x58), so the view
        // whose FollowCamera sits on the main camera IS the local player. The
        // Transform-chain test misses it because the camera is repositioned, not
        // parented, which is why the own body kept drawing at 11.8 m from the eye.
        void *g_entityViewType = nullptr;
        size_t g_offFollowCamera = static_cast<size_t>(-1);

        // Баг 2.1: возвращает расстояние от FollowCamera до g_cameraPosition,
        // или 1.0e9f если определить невозможно. Scan() затем выставит
        // selfView=true только одному слоту с минимальной дистанцией.
        float SelfFollowCameraDistance(void *component)
        {
            void *includeInactive = nullptr;
            (void)includeInactive;
            if (component == nullptr || m_getComponentInParent == nullptr || !g_cameraPositionValid)
            {
                static bool traced = false;
                if (!traced)
                {
                    traced = true;
                    Trace(component == nullptr            ? "self: no component"
                          : m_getComponentInParent == nullptr ? "self: no GetComponentInParent"
                                                              : "self: no camera position");
                }
                return 1.0e9f;
            }
            if (g_entityViewType == nullptr)
            {
                void *klass = g_resolver->FindClassAnyImage(
                    "Game.Battle.EntityModel.BattleCharacter.Views", "BattleCharacterEntityView");
                if (klass == nullptr)
                {
                    return 1.0e9f;   // retried on the next scan, cheap
                }
                g_entityViewType = g_resolver->TypeObjectOf(klass);
                g_offFollowCamera = g_resolver->FieldOffset(klass, "<FollowCamera>k__BackingField");
                if (g_offFollowCamera == static_cast<size_t>(-1))
                {
                    g_offFollowCamera = g_resolver->FieldOffset(klass, "FollowCamera");
                }
                Trace(g_entityViewType != nullptr && g_offFollowCamera != static_cast<size_t>(-1)
                          ? "self: follow camera resolved"
                          : "self: follow camera missing");
                if (g_entityViewType == nullptr || g_offFollowCamera == static_cast<size_t>(-1))
                {
                    return 1.0e9f;
                }
            }
            bool inactive = true;
            void *view = Invoke2(m_getComponentInParent, component, g_entityViewType, &inactive);
            if (view == nullptr || !Alive(view))
            {
                return 1.0e9f;
            }
            void *follow = ReadPointer(view, g_offFollowCamera);
            if (follow == nullptr)
            {
                return 1.0e9f;
            }
            Vec3 position{};
            if (!TransformPosition(follow, position))
            {
                return 1.0e9f;
            }
            // Возвращаем точное расстояние: Scan() выберет ОДИН слот-победитель.
            return aim::Length(aim::Sub(position, g_cameraPosition));
        }

        // Lowercase, space-separated words of a transform name. "Bip001 L
        // UpperArm", "L_UpperLeg" and "mixamorig:LeftUpLeg" become token lists,
        // because every rig spells the same joint differently and a substring
        // match on the raw name misses all the suffix-side spellings
        // ("Thigh_L", "Foot_L"), which is why the legs were never found.
        void NormaliseName(const char *name, char *out, size_t capacity)
        {
            size_t length = 0;
            bool previousLower = false;
            bool previousDigit = false;
            for (const char *cursor = name; *cursor != '\0' && length + 3 < capacity; ++cursor)
            {
                const char ch = *cursor;
                const bool upper = ch >= 'A' && ch <= 'Z';
                const bool lower = ch >= 'a' && ch <= 'z';
                const bool digit = ch >= '0' && ch <= '9';
                if (!upper && !lower && !digit)
                {
                    if (length > 0 && out[length - 1] != ' ')
                    {
                        out[length++] = ' ';
                    }
                    previousLower = false;
                    previousDigit = false;
                    continue;
                }
                // camelCase and digit boundaries are word boundaries too.
                if (length > 0 && out[length - 1] != ' ' &&
                    ((upper && previousLower) || (digit && previousLower) ||
                     (!digit && previousDigit)))
                {
                    out[length++] = ' ';
                }
                out[length++] = upper ? static_cast<char>(ch - 'A' + 'a') : ch;
                previousLower = lower;
                previousDigit = digit;
            }
            out[length] = '\0';
        }

        // Whole-word lookup in the normalised buffer.
        bool HasWord(const char *words, const char *word)
        {
            const size_t length = std::strlen(word);
            for (const char *hit = std::strstr(words, word); hit != nullptr;
                 hit = std::strstr(hit + 1, word))
            {
                const bool head = hit == words || hit[-1] == ' ';
                const bool tail = hit[length] == '\0' || hit[length] == ' ';
                if (head && tail)
                {
                    return true;
                }
            }
            return false;
        }

        // Substring lookup: catches names that are a single glued word
        // ("leftupperleg") and therefore never split into tokens.
        bool HasText(const char *words, const char *text)
        {
            return std::strstr(words, text) != nullptr;
        }

        bool EndsWith(const char *text, const char *suffix)
        {
            const size_t textLength = std::strlen(text);
            const size_t suffixLength = std::strlen(suffix);
            return textLength >= suffixLength &&
                   std::strcmp(text + textLength - suffixLength, suffix) == 0;
        }

        // +1 left, -1 right, 0 unknown. The single-letter form has to be a whole
        // word or "leg" would answer for "l".
        int SideOf(const char *words)
        {
            if (HasText(words, "left") || HasWord(words, "l"))
            {
                return 1;
            }
            if (HasText(words, "right") || HasWord(words, "r"))
            {
                return -1;
            }
            return 0;
        }

        // First standalone single digit in the normalised name, -1 when absent.
        // This rig numbers its chains ("l arm 2 jnt", "r leg 1 jnt"), and the
        // digit is the only thing that tells the elbow from the wrist: without
        // it all four arm joints answer "arm" and collapse into one.
        int SegmentIndex(const char *words)
        {
            for (const char *cursor = words; *cursor != '\0'; ++cursor)
            {
                if (*cursor >= '0' && *cursor <= '9' && (cursor == words || cursor[-1] == ' ') &&
                    (cursor[1] == '\0' || cursor[1] == ' '))
                {
                    return *cursor - '0';
                }
            }
            return -1;
        }

        // Skeleton joints drawn by the stick figure. The id is the
        // UModelerX/Unity HumanBodyBones index, so the same table drives both
        // the Animator path and the name walk.
        int MatchBone(const char *name)
        {
            char words[128];
            NormaliseName(name, words, sizeof(words));
            const int side = SideOf(words);
            const bool left = side > 0;
            const bool right = side < 0;

            const auto part = [&](const char *text) { return HasText(words, text); };
            const auto pair = [&](const char *a, const char *b) {
                return HasWord(words, a) && HasWord(words, b);
            };
            const auto sided = [&](int leftBone, int rightBone) {
                return left ? leftBone : right ? rightBone : -1;
            };

            // A head marker / head end cap is not the head bone. "forehead" is
            // the trap: it contains "head", it ends in _jnt, and it sits ABOVE
            // M_head_1_jnt on the female skin — so the crown circle was drawn on
            // the brow and the jaw (evidence: skelet/жен.txt, node [40]).
            if (part("forehead") || part("phantom") || part("assist") || part("twist"))
            {
                return -1;
            }
            if (part("head") && !part("top") && !part("end") && !part("jaw") && !part("eye") &&
                !part("hair"))
            {
                return BoneHead;
            }
            if (part("neck"))
            {
                return BoneNeck;
            }
            if (part("hips") || part("pelvis"))
            {
                return BoneHips;
            }
            // Numbered chains first: "l arm 0 jnt" (clavicle), "l arm 1 jnt"
            // (upper arm), "l arm 2 jnt" (elbow), "l arm 3 jnt" (wrist); the legs
            // are "l leg 1..3 jnt" (thigh, shin, ankle) plus "l foot 1..2 jnt".
            const int segment = SegmentIndex(words);
            if (segment >= 0)
            {
                if (part("arm"))
                {
                    if (segment == 0)
                    {
                        return sided(BoneLeftShoulder, BoneRightShoulder);
                    }
                    if (segment == 1)
                    {
                        return sided(BoneLeftUpperArm, BoneRightUpperArm);
                    }
                    if (segment == 2)
                    {
                        return sided(BoneLeftLowerArm, BoneRightLowerArm);
                    }
                    return sided(BoneLeftHand, BoneRightHand);
                }
                if (part("leg"))
                {
                    if (segment <= 1)
                    {
                        return sided(BoneLeftUpperLeg, BoneRightUpperLeg);
                    }
                    if (segment == 2)
                    {
                        return sided(BoneLeftLowerLeg, BoneRightLowerLeg);
                    }
                    return sided(BoneLeftFoot, BoneRightFoot);
                }
                if (part("foot") || part("ankle"))
                {
                    return sided(BoneLeftFoot, BoneRightFoot);
                }
            }
            if (part("clavicle") || part("shoulder"))
            {
                return sided(BoneLeftShoulder, BoneRightShoulder);
            }
            if (part("hand") || part("wrist"))
            {
                return sided(BoneLeftHand, BoneRightHand);
            }
            if (part("elbow") || part("forearm") || part("lowerarm") || pair("lower", "arm"))
            {
                return sided(BoneLeftLowerArm, BoneRightLowerArm);
            }
            if (part("upperarm") || part("armupper") || pair("upper", "arm"))
            {
                return sided(BoneLeftUpperArm, BoneRightUpperArm);
            }
            if (part("foot") || part("ankle"))
            {
                return sided(BoneLeftFoot, BoneRightFoot);
            }
            if (part("knee") || part("calf") || part("shin") || part("lowerleg") ||
                pair("lower", "leg"))
            {
                return sided(BoneLeftLowerLeg, BoneRightLowerLeg);
            }
            if (part("thigh") || part("upperleg") || part("upleg") || pair("upper", "leg"))
            {
                return sided(BoneLeftUpperLeg, BoneRightUpperLeg);
            }
            if (part("chest") || part("torso") || part("spinetop"))
            {
                return BoneChest;
            }
            // "M_spine_1_jnt" is this rig's pelvis: the characters ship without a
            // node called hips, so the first spine segment has to carry the hips
            // slot or the whole lower half draws off the box.
            if (part("spine") || part("waist") || part("belly"))
            {
                if (segment == 1)
                {
                    return BoneHips;
                }
                if (segment >= 3)
                {
                    return BoneChest;
                }
                return BoneSpine;
            }
            // Last resort: a bare side+limb name. "LeftLeg" is read as the thigh
            // because a chain needs a root; the lower joint then wins by height.
            if (part("arm"))
            {
                return sided(BoneLeftUpperArm, BoneRightUpperArm);
            }
            if (part("leg"))
            {
                return sided(BoneLeftUpperLeg, BoneRightUpperLeg);
            }
            return -1;
        }

        // One-shot dump of every transform name under the first character, so the
        // bone pattern table can be written against the names this game actually
        // uses instead of guessed ones. Written once per process.
        bool g_rigNamesDumped = false;

        // One native Transform calibration per process: which offset inside the
        // native block holds the world position. Armed by an empty file in
        // %TEMP%, so the probe costs nothing in a normal run.
        bool g_nativeProbeDone = false;

        void WriteDebugFile(const wchar_t *fileName, const std::string &body)
        {
            wchar_t path[MAX_PATH]{};
            if (GetTempPathW(MAX_PATH, path) == 0)
            {
                return;
            }
            lstrcatW(path, fileName);
            HANDLE file = CreateFileW(path, GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS,
                                      FILE_ATTRIBUTE_NORMAL, nullptr);
            if (file == INVALID_HANDLE_VALUE)
            {
                return;
            }
            DWORD written = 0;
            WriteFile(file, body.data(), static_cast<DWORD>(body.size()), &written, nullptr);
            CloseHandle(file);
        }

        // Unboxes a boxed int (Transform.get_childCount returns int32).
        int UnboxInt(void *boxed)
        {
            if (boxed == nullptr)
            {
                return -1;
            }
            const void *value = g_resolver->Unbox(boxed);
            if (value == nullptr)
            {
                return -1;
            }
            int result = -1;
            std::memcpy(&result, value, sizeof(result));
            return result;
        }

        // Generic rig: walk the character's transform tree and pick the joints out
        // by name. Runs once per character, inside one tick, and is bounded: a
        // broken hierarchy costs a capped walk, not a stall.
        //
        // The child list comes from Transform.get_childCount / GetChild(int)
        // rather than GetComponentsInChildren(Type,bool): the Type overload throws
        // inside this runtime, and every thrown call stands the whole ESP down for
        // five seconds because the thread's il2cpp state does not survive it.

        struct JointCandidate
        {
            int bone = -1;
            void *node = nullptr;
            // Parent in the Transform walk. Free: the walk already knows it.
            // It is what makes an arm chain provably one arm (shoulder ->
            // upper arm -> lower arm -> hand is a parent chain in this rig),
            // so a tier can prefer a joint whose ancestor is the tier above
            // instead of accepting whatever answered the same name.
            void *parent = nullptr;
            void *rig = nullptr;  // the mainRigBody this joint was found under
            Vec3 pos{};
            bool isJnt = false;  // named "..._jnt": this rig's skeleton convention
            bool headIsBase = false;  // strict table: head is the skull base, not the crown
            char name[40] = "";
        };
        // Bounded: a rig can expose hundreds of matching nodes (LODs, twist
        // bones, mirrors) and the walk must not grow with them.
        constexpr size_t kMaxJointCandidates = 160;
        constexpr int kCandidatesPerBone = 8;

        // A joint name is not unique in a scene, a position is. The measured
        // rig is 0.5 m wide at the wrists (l arm 0 at dx -0.29, r arm 3 at
        // +0.22 in OmegaCore_rig.txt) and 1.9 m tall, so a node that answers to
        // the right name but sits past this box around the body's own feet
        // belongs to the player standing next to it. This is the gate that
        // stops a skeleton from borrowing a neighbour's arm.
        //
        // 1.60 m, not 0.85: a body seated in a car folds its knees and feet
        // forward of the root, and a character that just stepped out still has
        // one leg inside the door. At 0.85 the legs fell outside the box and
        // PickJoints handed back a limbless torso - the skeleton that only came
        // back after a few seconds of walking. The tighter gates in BonePass
        // (envelope from hips/chest, torso rigidity) still reject a real
        // neighbour.
        constexpr float kRigHorizontalRadius = 1.60f;
        constexpr float kRigBelowFeet = 1.70f;   // seated or crouched legs
        constexpr float kRigAboveFeet = 2.60f;   // an arm raised over the head
        // A cached rig that reads nothing within this radius of the body it is
        // drawn on is not that body's rig at all (a pooled view comes back with
        // a different character under the same component pointer).
        constexpr float kRigLostDistance = 3.00f;
        // Below this many "..._jnt" nodes the rig is not using the convention and
        // the wider name match stays in play.
        constexpr int kJntRigMinimum = 8;

        // Where a joint sits on the body as a fraction of the ground-to-top
        // height. These are the only numbers the joint choice depends on, and
        // they are anatomy, not game offsets. A candidate further than the
        // tolerance from its slot is not that joint and is not drawn.
        constexpr float kJointHeightTolerance = 0.16f;

        float ExpectedHeight(int bone)
        {
            switch (bone)
            {
            // The CROWN, so the box top sits on the top of the skull and the head
            // circle wraps the skull (it is drawn one radius below this joint).
            // This rig offers "m head 1 jnt" (skull base), "m forehead dn jnt"
            // (brow) and "m head 2 jnt" (crown); only the crown is exact here.
            case BoneHead: return 1.00f;
            case BoneNeck: return 0.88f;
            case BoneChest: return 0.80f;
            case BoneSpine: return 0.66f;
            case BoneHips: return 0.53f;
            case BoneLeftShoulder:
            case BoneRightShoulder: return 0.86f;
            case BoneLeftUpperArm:
            case BoneRightUpperArm: return 0.82f;
            case BoneLeftLowerArm:
            case BoneRightLowerArm: return 0.68f;
            case BoneLeftHand:
            case BoneRightHand: return 0.55f;
            case BoneLeftUpperLeg:
            case BoneRightUpperLeg: return 0.50f;
            case BoneLeftLowerLeg:
            case BoneRightLowerLeg: return 0.27f;
            case BoneLeftFoot:
            case BoneRightFoot: return 0.04f;
            default: return 0.5f;
            }
        }

        // The rig ships with fixed joint names and every one of the nineteen
        // hangs under a single "mainRigBody" node (evidence: skelet/жен.txt and
        // skelet/муж.txt, full hierarchies dumped from the game). Females carry
        // no crown node at all — M_head_2_jnt is male-only and the female chain
        // ends in M_hair_1_jnt — so "head" is the skull base there and the
        // crown has to be estimated. Substring matching cannot see that: it
        // reads "m forehead dn jnt" as a head and puts the circle on the face.
        int ExactRigBone(const char *words, bool &headIsBase)
        {
            headIsBase = false;
            struct Entry
            {
                const char *name;
                int bone;
                bool base;
            };
            static const Entry kTable[] = {
                {"m spine 1 jnt", BoneHips, false},
                {"m spine 2 jnt", BoneSpine, false},
                {"m spine 3 jnt", BoneChest, false},
                {"m neck jnt", BoneNeck, false},
                {"m head 2 jnt", BoneHead, false},
                {"m head 1 jnt", BoneHead, true},
                {"l arm 0 jnt", BoneLeftShoulder, false},
                {"l arm 1 jnt", BoneLeftUpperArm, false},
                {"l arm 2 jnt", BoneLeftLowerArm, false},
                {"l arm 3 jnt", BoneLeftHand, false},
                {"r arm 0 jnt", BoneRightShoulder, false},
                {"r arm 1 jnt", BoneRightUpperArm, false},
                {"r arm 2 jnt", BoneRightLowerArm, false},
                {"r arm 3 jnt", BoneRightHand, false},
                {"l leg 1 jnt", BoneLeftUpperLeg, false},
                {"l leg 2 jnt", BoneLeftLowerLeg, false},
                {"l leg 3 jnt", BoneLeftFoot, false},
                {"r leg 1 jnt", BoneRightUpperLeg, false},
                {"r leg 2 jnt", BoneRightLowerLeg, false},
                {"r leg 3 jnt", BoneRightFoot, false},
            };
            for (const Entry &entry : kTable)
            {
                if (std::strcmp(words, entry.name) == 0)
                {
                    headIsBase = entry.base;
                    return entry.bone;
                }
            }
            return -1;
        }

        // Branches of the character that never carry a skeleton joint: the
        // phantom duplicate rig, the aim-assist markers, weapon and magazine
        // holders, the bag, the face and finger chains, twist bones, foot
        // helpers and every LOD mesh. Descending into them costs managed calls
        // (350 budget, ~30 joints actually needed) and lets a same-named node
        // from another rig into the candidate list.
        bool ShouldPruneSubtree(const char *words)
        {
            // Tokens must match what NormaliseName() actually produces: it
            // splits camelCase AND digit runs, so the real names arrive as
            // "aim assist", "l arm twist", "player camera", "l head 01". The
            // glued spellings that used to live here ("aimassist",
            // "l armtwist", "playercamera", "l head01") never matched a single
            // node, so the walk descended into ~110 useless nodes per character.
            static const char *const kPrefixes[] = {
                "phantom", "aim assist", "battle assistance", "weapon", "magazine",
                "cargo parent", "bag", "l bag", "r bag", "keychain", "target collider",
                "player sound", "footstep", "look rotation", "player camera",
                "player weapon", "l arm twist", "r arm twist", "l foot", "r foot",
                "l index", "l middle", "l ring", "l pinky", "l thumb",
                "r index", "r middle", "r ring", "r pinky", "r thumb",
                "l eyelid", "r eyelid", "l head 01", "r head 01", "m lips", "l lips",
                "r lips", "m jaw", "m tongue", "l nose", "r nose", "l cheek",
                "r cheek", "l eyebrow", "r eyebrow", "m forehead", "m hair",
                "f mask", "f hair", "f bald", "f beards", "f body", "f strap",
                "f head", "m mask", "m body", "m strap", "m bag", "l head",
                "r head",
            };
            for (const char *prefix : kPrefixes)
            {
                if (std::strncmp(words, prefix, std::strlen(prefix)) == 0)
                {
                    return true;
                }
            }
            // Every LOD container / mesh ends with "lods"/"lod0".."lod4" or
            // carries " lod" in the middle of the name.
            const size_t length = std::strlen(words);
            if (length >= 4 && std::strcmp(words + length - 4, "lods") == 0)
            {
                return true;
            }
            if (length >= 4 && std::strncmp(words + length - 4, "lod", 3) == 0 &&
                words[length - 1] >= '0' && words[length - 1] <= '9')
            {
                return true;
            }
            return std::strstr(words, " lod") != nullptr;
        }

        // One node per joint, chosen by where the node sits on the body: the
        // highest matched node is the top of the character, the lowest is the
        // ground, and every joint has a known fraction between the two. Side
        // comes from the sign of X against the character's own centre, so a rig
        // that spells left and right the other way round still pairs up.
        void PickJoints(Slot &slot, const std::vector<JointCandidate> &candidates)
        {
            for (int bone = 0; bone < kBoneCount; ++bone)
            {
                slot.bone[bone] = nullptr;
                slot.boneHold[bone] = 0;
            }
            if (candidates.empty())
            {
                return;
            }

            // The name walk starts at this character's model root, but that root
            // is not guaranteed: a climb that lands on a shared container walks
            // into the next body, and a body that carries two rigs (a weapon
            // holder animator, a cloth rig) answers twice. Matching by name then
            // hands this skeleton a joint of the neighbour - the arm that copies
            // the rifle pose of the player standing beside it. Position decides:
            // every candidate has to sit in a box around the feet this slot
            // already sampled.
            //   Fail-open on purpose. A missing or stale feet anchor skips the
            // gate, and a gate that would leave fewer joints than a humanoid
            // skeleton falls back to the unfiltered list, so a bad anchor costs
            // accuracy but never the whole figure.
            std::vector<JointCandidate> nearby;
            nearby.reserve(candidates.size());
            // Read the anchor NOW instead of trusting the position sampled for
            // the draw list. Between the scan and this walk a body steps out of a
            // car, gets seated, or simply walks; the stale anchor then sits
            // metres from the joints it is meant to gate, the gate throws the
            // whole rig away, and the character stays boneless until the next
            // retry - that is the "no skeleton right after leaving the car"
            // report. The fresh position is stamped back, so the next gate has a
            // live anchor even on a slot the sampler skipped.
            Vec3 liveFeet{};
            if (slot.transform != nullptr && TransformPosition(slot.transform, liveFeet))
            {
                slot.lastFeet = liveFeet;
                slot.haveFeet = true;
                slot.feetTick = GetTickCount64();
            }
            const bool haveAnchor = slot.haveFeet && slot.feetTick != 0 &&
                                    GetTickCount64() - slot.feetTick <= 2000;
            if (haveAnchor)
            {
                const Vec3 &anchor = slot.lastFeet;
                for (const JointCandidate &candidate : candidates)
                {
                    const float dx = candidate.pos.x - anchor.x;
                    const float dy = candidate.pos.y - anchor.y;
                    const float dz = candidate.pos.z - anchor.z;
                    if (dx * dx + dz * dz > kRigHorizontalRadius * kRigHorizontalRadius)
                    {
                        continue;
                    }
                    if (dy < -kRigBelowFeet || dy > kRigAboveFeet)
                    {
                        continue;
                    }
                    nearby.push_back(candidate);
                }
            }
            // Fail closed on a live anchor. A pooled view component comes back
            // under a different character, and the unfiltered list then holds
            // the bones of the body standing beside this one - the arm that
            // copies a neighbour's rifle pose. Nothing sits near this body's own
            // feet means there is nothing of this body to draw; the rig is
            // retried rather than borrowed.
            if (haveAnchor && nearby.size() < static_cast<size_t>(kHumanoidJointThreshold))
            {
                slot.rigCooldown = GetTickCount64() + 500;
                return;
            }
            const std::vector<JointCandidate> &source = nearby.empty() ? candidates : nearby;

            float lowY = source[0].pos.y;
            float highY = source[0].pos.y;
            bool jntOnly = true;
            for (const JointCandidate &candidate : source)
            {
                lowY = std::min(lowY, candidate.pos.y);
                highY = std::max(highY, candidate.pos.y);
                jntOnly = jntOnly && candidate.isJnt;
            }
            const float height = std::max(highY - lowY, 0.2f);

            for (int bone = 0; bone < kBoneCount; ++bone)
            {
                const float want = ExpectedHeight(bone);
                float best = 1.0e9f;
                for (const JointCandidate &candidate : source)
                {
                    if (candidate.bone != bone)
                    {
                        continue;
                    }
                    // A matching name is not yet a joint. This rig hangs phantom
                    // and hit-box nodes under the character (a node called
                    // "head" at the feet drew the whole figure upside down), so a
                    // candidate that sits far from where anatomy puts this joint
                    // is dropped instead of being drawn.
                    const float fraction = (candidate.pos.y - lowY) / height;
                    const float distance = std::fabs(fraction - want);
                    // Leg joints skip this test: a character seated in a car has
                    // the thigh BELOW the knee, so a height band taken from a
                    // standing figure rejects exactly the joints that are there.
                    // The leg chains are rebuilt from the rig's own numbering
                    // below, which does not care about the pose.
                    const bool isLeg = bone == BoneLeftUpperLeg || bone == BoneRightUpperLeg ||
                                       bone == BoneLeftLowerLeg || bone == BoneRightLowerLeg ||
                                       bone == BoneLeftFoot || bone == BoneRightFoot;
                    if (!isLeg && !jntOnly && distance > kJointHeightTolerance)
                    {
                        continue;
                    }
                    // Side comes from the name only (SideOf already parsed the
                    // l_/r_ prefix into candidate.bone). The old world-x test
                    // assumed the character always faces along Z, so a body
                    // turned 90 degrees had its arms swapped across the chest.
                    float score = distance;
                    if (score < best)
                    {
                        best = score;
                        slot.bone[bone] = candidate.node;
                    }
                }
            }

            // Arms and legs are rebuilt from the rig's own numbering, not from
            // height and not from world x.
            //
            // Height is unusable: a character seated in a car has the thigh BELOW
            // the knee, and the old "sort by Y" chain then drew the leg folded
            // backwards. The number in the name is the chain position and never
            // moves with the pose. The l/r letter picks the side and nothing else
            // does: a world-x flip put the elbow across the chest the moment a
            // body turned to face along X.
            {
                // tier <- bone index pairs, per side, per limb.
                const int armLeft[4] = {BoneLeftShoulder, BoneLeftUpperArm, BoneLeftLowerArm,
                                        BoneLeftHand};
                const int armRight[4] = {BoneRightShoulder, BoneRightUpperArm, BoneRightLowerArm,
                                         BoneRightHand};
                const int legLeft[3] = {BoneLeftUpperLeg, BoneLeftLowerLeg, BoneLeftFoot};
                const int legRight[3] = {BoneRightUpperLeg, BoneRightLowerLeg, BoneRightFoot};

                // Is `ancestor` one of the three nodes above `node` in the
                // Transform tree? Joints of one limb are a parent chain in this
                // rig, so this answers "same arm" without trusting the name.
                const auto hasAncestor = [&source](void *node, void *ancestor) {
                    if (node == nullptr || ancestor == nullptr)
                    {
                        return false;
                    }
                    void *walk = node;
                    for (int up = 0; up < 3; ++up)
                    {
                        void *parent = nullptr;
                        bool known = false;
                        for (const JointCandidate &candidate : source)
                        {
                            if (candidate.node == walk)
                            {
                                parent = candidate.parent;
                                known = true;
                                break;
                            }
                        }
                        if (!known || parent == nullptr)
                        {
                            return false;
                        }
                        if (parent == ancestor)
                        {
                            return true;
                        }
                        walk = parent;
                    }
                    return false;
                };

                const auto rebuild = [&](const int *leftBones, const int *rightBones, int tiers) {
                    for (int t = 0; t < tiers; ++t)
                    {
                        slot.bone[leftBones[t]] = nullptr;
                        slot.bone[rightBones[t]] = nullptr;
                    }
                    // Side 0 is the chain whose names carry the l prefix, side 1
                    // the r prefix — exactly what the rig itself says. No world
                    // coordinate takes part in this decision.
                    for (int side = 0; side < 2; ++side)
                    {
                        const int *bones = side == 0 ? leftBones : rightBones;
                        void *chosen = nullptr;
                        for (int tier = 0; tier < tiers; ++tier)
                        {
                            const JointCandidate *best = nullptr;
                            int bestRank = 9;
                            float bestDist = 1.0e9f;
                            bool bestChained = false;
                            for (const JointCandidate &candidate : source)
                            {
                                if (candidate.bone != bones[tier])
                                {
                                    continue;
                                }
                                int rank = 0;
                                if (std::strstr(candidate.name, "twist") != nullptr)
                                {
                                    rank = 2;
                                }
                                else if (std::strstr(candidate.name, "foot") != nullptr ||
                                         std::strstr(candidate.name, "finger") != nullptr)
                                {
                                    rank = 1;
                                }
                                // No anchor, no tiebreak: fall back to walk order,
                                // which is what this code always did.
                                float distSq = 0.0f;
                                if (slot.haveFeet)
                                {
                                    const float dx = candidate.pos.x - slot.lastFeet.x;
                                    const float dz = candidate.pos.z - slot.lastFeet.z;
                                    distSq = dx * dx + dz * dz;
                                }
                                const bool chained = hasAncestor(candidate.node, chosen);
                                bool better = false;
                                if (best == nullptr)
                                {
                                    better = true;
                                }
                                else if (chained != bestChained)
                                {
                                    // A joint hanging off the tier above is the
                                    // same limb. The same name on a neighbour,
                                    // on a second weapon rig or on a phantom
                                    // skeleton is not, however close it sits.
                                    better = chained;
                                }
                                else if (rank != bestRank)
                                {
                                    better = rank < bestRank;
                                }
                                else
                                {
                                    better = distSq < bestDist;
                                }
                                if (better)
                                {
                                    best = &candidate;
                                    bestChained = chained;
                                    bestRank = rank;
                                    bestDist = distSq;
                                }
                            }
                            if (best != nullptr)
                            {
                                slot.bone[bones[tier]] = best->node;
                                chosen = best->node;
                            }
                            else
                            {
                                chosen = nullptr;
                            }
                        }
                    }
                };

                rebuild(legLeft, legRight, 3);
                rebuild(armLeft, armRight, 4);

                // Rigid segments. A bone cannot change length when the pose
                // changes, so the distance between neighbouring joints of one
                // limb is a constant of that rig. Every gate above measures from
                // the feet or from the chest, and a body standing 0.8 m away
                // passes all of them - this one does not: a chain stitched from
                // two characters fails at the link that spans the gap between
                // them, which is the "his arm suddenly posed like the player
                // with the rifle" frame. Segments are checked, never corrected:
                // a broken link drops that tier and everything below it, so the
                // figure keeps the shoulder it owns.
                const auto positionOf = [&source](void *node, Vec3 &out) {
                    for (const JointCandidate &candidate : source)
                    {
                        if (candidate.node == node)
                        {
                            out = candidate.pos;
                            return true;
                        }
                    }
                    return false;
                };
                const int *armTiers[2] = {armLeft, armRight};
                int brokenTier[2] = {-1, -1};
                float brokenLength[2] = {0.0f, 0.0f};
                for (int side = 0; side < 2; ++side)
                {
                    Vec3 previous{};
                    bool havePrevious = slot.bone[BoneChest] != nullptr &&
                                        positionOf(slot.bone[BoneChest], previous);
                    for (int tier = 0; tier < 4; ++tier)
                    {
                        Vec3 current{};
                        if (slot.bone[armTiers[side][tier]] == nullptr ||
                            !positionOf(slot.bone[armTiers[side][tier]], current))
                        {
                            havePrevious = false;
                            continue;
                        }
                        if (havePrevious)
                        {
                            const float length = aim::Length(aim::Sub(current, previous));
                            const float limit = tier == 0 ? 0.55f : 0.60f;
                            if (aim::Finite(length) && length > limit)
                            {
                                brokenTier[side] = tier;
                                brokenLength[side] = length;
                                break;
                            }
                        }
                        previous = current;
                        havePrevious = true;
                    }
                }
                // Both sides failing at the shoulder means the chest itself came
                // from the wrong body, not the arms - dropping both arms would
                // hide a figure that is merely mis-anchored.
                if (brokenTier[0] == 0 && brokenTier[1] == 0)
                {
                    brokenTier[0] = -1;
                    brokenTier[1] = -1;
                }
                for (int side = 0; side < 2; ++side)
                {
                    if (brokenTier[side] < 0)
                    {
                        continue;
                    }
                    char line[128];
                    std::snprintf(line, sizeof(line),
                                  "rig: arm link %.2f m at tier %d, limb dropped",
                                  brokenLength[side], brokenTier[side]);
                    TraceLine(line);
                    for (int k = brokenTier[side]; k < 4; ++k)
                    {
                        slot.bone[armTiers[side][k]] = nullptr;
                    }
                }
            }
        }

        // Where the world position of a joint actually lives in Unity's native
        // Transform. The managed walk already yields the position, so the layout
        // is CALIBRATED against it instead of guessed: read the native block,
        // find the three floats that equal the position we asked the engine for,
        // and that offset is the answer for this build. One sentinel, one dump.
        //
        // This is the step that makes limbs cheap: a limb read becomes one
        // ReadProcessMemory on a cached native transform, with no managed invoke
        // on the render thread at all — the same shape as reading a native bone
        // array by index in a Source engine title.
        void ProbeNativeJoints(const std::vector<JointCandidate> &candidates)
        {
            wchar_t path[MAX_PATH]{};
            if (GetTempPathW(MAX_PATH, path) == 0)
            {
                return;
            }
            lstrcatW(path, L"OmegaCore_nativeprobe");
            if (GetFileAttributesW(path) == INVALID_FILE_ATTRIBUTES)
            {
                return;
            }

            constexpr int kWindow = 0x160;
            std::string body = "-- native transform probe --\n";
            int probed = 0;
            for (const JointCandidate &candidate : candidates)
            {
                if (candidate.node == nullptr || ++probed > 8)
                {
                    break;
                }
                // UnityEngine.Object.m_CachedPtr, dump.cs "private IntPtr
                // m_CachedPtr; // 0x10" — the native object a wrapper points at.
                void *native = nullptr;
                SIZE_T got = 0;
                if (!ReadProcessMemory(GetCurrentProcess(),
                                       static_cast<unsigned char *>(candidate.node) + 0x10, &native,
                                       sizeof(native), &got) ||
                    got != sizeof(native) || native == nullptr)
                {
                    continue;
                }
                unsigned char block[kWindow]{};
                if (!ReadProcessMemory(GetCurrentProcess(), native, block, sizeof(block), &got) ||
                    got < 64)
                {
                    continue;
                }
                char line[192];
                std::snprintf(line, sizeof(line), "%s native=0x%llX managed=(%.3f %.3f %.3f)\n",
                              candidate.name,
                              static_cast<unsigned long long>(
                                  reinterpret_cast<uintptr_t>(native)),
                              candidate.pos.x, candidate.pos.y, candidate.pos.z);
                body += line;
                for (int off = 0; off + 12 <= kWindow; off += 4)
                {
                    float value[3];
                    std::memcpy(value, block + off, sizeof(value));
                    if (std::fabs(value[0] - candidate.pos.x) < 0.002f &&
                        std::fabs(value[1] - candidate.pos.y) < 0.002f &&
                        std::fabs(value[2] - candidate.pos.z) < 0.002f)
                    {
                        std::snprintf(line, sizeof(line), "  match at +0x%X\n", off);
                        body += line;
                    }
                }
            }
            WriteDebugFile(L"OmegaCore_native.txt", body);
        }

        // Defined with the bone pass; the rig resolver needs it first to tell
        // its own mainRigBody from a neighbour's by the Unity parent chain.
        bool RigUnderTransform(void *transform, void *rigRoot);

        // True when slot.rigRoot really is a rig of THIS character. The view
        // component is not always a descendant of the character root — seated in
        // a car, or on a skin whose mainRigBody is a sibling of the view — so
        // ownership is measured against the node the name walk started from, and
        // the view is separately required to still live under it. Comparing the
        // rig against the view alone threw away perfectly good rigs (19/19 joints
        // found, then "does not hang under this body" every pass), which is the
        // skeleton that never came back at the trader or after leaving a car.
        bool RigOwnedBySlot(const Slot &slot)
        {
            if (slot.rigRoot == nullptr)
            {
                return false;
            }
            void *anchor = slot.charRoot != nullptr ? slot.charRoot : slot.transform;
            if (!RigUnderTransform(anchor, slot.rigRoot))
            {
                return false;
            }
            if (slot.charRoot != nullptr && slot.transform != nullptr &&
                slot.transform != slot.charRoot && !RigUnderTransform(slot.charRoot, slot.transform))
            {
                return false;
            }
            return true;
        }

        void ResolveRigByNames(Slot &slot)
        {
            // A joint set that already answered is the most valuable thing this
            // slot owns, and the walk below clears slot.bone[] before it looks.
            // One unlucky root then threw a live 19-joint rig away and left the
            // player boneless for the rest of the match: a body seated in a
            // vehicle answers `rig: 0/19 joints by position (1 nodes, 0 cand)`
            // because the animator transform is a leaf there, not the character.
            // Snapshot the rig and put it back whenever the walk comes up short.
            void *keptBone[kBoneCount];
            for (int bone = 0; bone < kBoneCount; ++bone)
            {
                keptBone[bone] = slot.bone[bone];
            }
            void *keptRigRoot = slot.rigRoot;
            const int keptHits = slot.boneHits;
            const auto restoreKept = [&]() -> bool {
                if (keptHits < kHumanoidJointThreshold || keptRigRoot == nullptr)
                {
                    return false;
                }
                int alive = 0;
                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    if (keptBone[bone] != nullptr && Alive(keptBone[bone]))
                    {
                        ++alive;
                    }
                }
                if (alive < kHumanoidJointThreshold)
                {
                    return false;
                }
                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    slot.bone[bone] = keptBone[bone];
                }
                slot.rigRoot = keptRigRoot;
                slot.boneHits = keptHits;
                slot.bonesResolved = true;
                slot.rigCooldown = 0;
                // The figure moved (a seat, a door) but the nodes are the same
                // ones, so the next pass judges them against the live anchor
                // instead of restamping a baseline that hides a real jump.
                slot.boneBaseline = false;
                TraceLine("rig: kept the previous joint set (the name walk found none)");
                return true;
            };
            if (g_transformClass == nullptr)
            {
                g_transformClass = g_resolver->FindClassAnyImage("UnityEngine", "Transform");
            }
            if (m_getChildCount == nullptr && g_transformClass != nullptr)
            {
                m_getChildCount = g_resolver->Method(g_transformClass, "get_childCount", 0);
            }
            if (m_getChild == nullptr && g_transformClass != nullptr)
            {
                m_getChild = g_resolver->Method(g_transformClass, "GetChild", 1);
            }
            if (m_getParent == nullptr && g_transformClass != nullptr)
            {
                m_getParent = g_resolver->Method(g_transformClass, "get_parent", 0);
            }
            if (m_getChildCount == nullptr || m_getChild == nullptr || m_getName == nullptr)
            {
                Trace("rig: name walk unavailable");
                slot.bonesResolved = true;
                return;
            }

            void *root = nullptr;
            if (slot.animator != nullptr && m_getTransform != nullptr && Alive(slot.animator))
            {
                root = Invoke1(m_getTransform, slot.animator, nullptr);
            }
            if (root == nullptr)
            {
                root = slot.transform;
            }
            if (root == nullptr)
            {
                if (!restoreKept())
                {
                    // Not `bonesResolved = true`: that was a permanent verdict
                    // on a body that had merely not been spawned into its rig
                    // yet, and it left the slot without a skeleton until the
                    // scene changed.
                    slot.bonesResolved = false;
                    slot.rigCooldown = GetTickCount64() + 1000;
                    Trace("rig: no root transform");
                }
                return;
            }

            // The scanned view usually sits on a leaf of the character
            // hierarchy, so walking down from it sees one or two nodes. Climb
            // while the chain is single-child to land on the model root, where
            // the skeleton actually branches.
            //
            // Two stops keep the climb inside the character: a name that is
            // already the character (Player_*, Man, Woman, mainRigBody), and a
            // parent with many children (a scene container). Without them a
            // character parented to a car seat - which has exactly one child,
            // the character - could climb straight into the vehicle hierarchy.
            for (int depth = 0; depth < 12 && m_getParent != nullptr; ++depth)
            {
                if (m_getName != nullptr)
                {
                    void *managed = Invoke1(m_getName, root, nullptr);
                    if (managed != nullptr)
                    {
                        const std::string text = g_resolver->ReadString(managed, 48);
                        if (text.rfind("Player_", 0) == 0 || text == "Man" ||
                            text == "Woman" || text == "mainRigBody")
                        {
                            break;
                        }
                    }
                }
                g_stage = "rig: childCount";
                const int children = UnboxInt(Invoke1(m_getChildCount, root, nullptr));
                if (children > 1)
                {
                    break;
                }
                g_stage = "rig: parent";
                void *parent = Invoke1(m_getParent, root, nullptr);
                if (parent == nullptr)
                {
                    break;
                }
                // Climbing into the vehicle is exactly where the rig is lost: a
                // character parented to a seat has one child all the way up, so
                // the single-child rule alone walks out of the body and starts
                // the DFS on wheels and body panels.
                if (m_getName != nullptr)
                {
                    char parentName[48] = "";
                    if (TransformNodeName(parent, parentName, sizeof(parentName)) &&
                        LooksLikeVehicleName(parentName))
                    {
                        break;
                    }
                }
                if (UnboxInt(Invoke1(m_getChildCount, parent, nullptr)) > 24)
                {
                    break;
                }
                root = parent;
            }
            // The node the joints were found from. Ownership checks compare the
            // rig against THIS, not against the view: a skin whose mainRigBody
            // is a sibling of the view is still this body's rig.
            slot.charRoot = root;

            // Every node whose name matched a joint is kept with its world
            // position. The name alone is not enough: this rig hangs LOD meshes,
            // aim-assist markers and a whole phantom skeleton under the same
            // character, and "shortest name that says head" landed on a node at
            // the feet. Position decides instead; see PickJoints.
            std::vector<JointCandidate> candidates;
            std::vector<JointCandidate> exactJoints;
            candidates.reserve(64);
            exactJoints.reserve(24);
            int candidatesPerBone[kBoneCount]{};

            std::vector<void *> pending;
            pending.reserve(64);
            pending.push_back(root);
            // Parent of every queued node, parallel to `pending`. The DFS
            // already knows which node produced which child, so the tree
            // relation costs nothing on top of the walk.
            std::vector<void *> pendingParent;
            pendingParent.reserve(64);
            pendingParent.push_back(nullptr);
            // The mainRigBody each queued node belongs to, so the strict table
            // can tell one body's joints from a neighbour's.
            std::vector<void *> pendingRig;
            pendingRig.reserve(64);
            pendingRig.push_back(nullptr);
            size_t visited = 0;
            std::string collected;

            while (!pending.empty() && visited < kRigWalkMaxNodes)
            {
                void *node = pending.back();
                pending.pop_back();
                void *nodeParent = pendingParent.back();
                pendingParent.pop_back();
                void *nodeRig = pendingRig.back();
                pendingRig.pop_back();
                ++visited;
                if (!Alive(node))
                {
                    continue;
                }

                bool pruned = false;
                if (m_getName != nullptr)
                {
                    g_stage = "rig: name";
                    void *managed = Invoke1(m_getName, node, nullptr);
                    if (managed != nullptr)
                    {
                        const std::string text = g_resolver->ReadString(managed, 48);
                        if (!text.empty())
                        {
                            char normalised[64];
                            NormaliseName(text.c_str(), normalised, sizeof(normalised));
                            // NormaliseName splits camelCase, so the real
                            // name arrives as "main rig body". The glued form
                            // is accepted too: this compare silently failed
                            // for a whole session and left nodeRig null on
                            // every node, which disabled the strict table and
                            // the per-body grouping below.
                            if (std::strcmp(normalised, "main rig body") == 0 ||
                                std::strcmp(normalised, "mainrigbody") == 0)
                            {
                                nodeRig = node;
                            }
                            // Pruning used to start only once the walk had
                            // entered a mainRigBody, but the expensive siblings
                            // (f_body_lods, f_head_grp, PlayerCameraRoot,
                            // PlayerWeaponAnimator, BagCharmController) sit NEXT
                            // to it, so ~110 nodes were still descended. The
                            // root itself is never pruned: it is the anchor.
                            if (node != root)
                            {
                                pruned = ShouldPruneSubtree(normalised);
                            }
                            if (!pruned)
                            {
                                bool headIsBase = false;
                                const int exactBone = ExactRigBone(normalised, headIsBase);
                                if (exactBone >= 0 && exactJoints.size() < kMaxJointCandidates)
                                {
                                    Vec3 position{};
                                    if (TransformPosition(node, position))
                                    {
                                        JointCandidate exact;
                                        exact.bone = exactBone;
                                        exact.node = node;
                                        exact.parent = nodeParent;
                                        exact.rig = nodeRig;
                                        exact.pos = position;
                                        exact.isJnt = true;
                                        exact.headIsBase = headIsBase;
                                        std::snprintf(exact.name, sizeof(exact.name), "%s",
                                                      normalised);
                                        exactJoints.push_back(exact);
                                    }
                                }
                                const int bone = MatchBone(normalised);
                                if (!g_rigNamesDumped && collected.size() < 262144)
                                {
                                    // Position on the same line: a name alone has
                                    // already put a "head" at the feet, so the dump
                                    // has to show where each node actually sits.
                                    Vec3 where{};
                                    const bool ok = TransformPosition(node, where);
                                    char line[160];
                                    std::snprintf(line, sizeof(line),
                                                  "y=%7.2f x=%7.2f z=%7.2f  %-36s -> %2d%s\n",
                                                  ok ? where.y : 0.0f, ok ? where.x : 0.0f,
                                                  ok ? where.z : 0.0f, normalised, bone,
                                                  ok ? "" : "  (posfail)");
                                    collected += line;
                                }
                                if (bone >= 0 && candidates.size() < kMaxJointCandidates)
                                {
                                    const bool isJnt = EndsWith(normalised, "jnt");
                                    bool room = candidatesPerBone[bone] < kCandidatesPerBone;
                                    int replaceAt = -1;
                                    if (!room && isJnt)
                                    {
                                        // The cap fills in walk order and this rig lists
                                        // its LOD meshes first, so a slot's eight places
                                        // can be all "m head 01 bald lod N" and the real
                                        // "m head 1 jnt" never enters the list. A joint
                                        // displaces a non-joint of the same bone.
                                        for (size_t k = 0; k < candidates.size(); ++k)
                                        {
                                            if (candidates[k].bone == bone && !candidates[k].isJnt)
                                            {
                                                replaceAt = static_cast<int>(k);
                                                break;
                                            }
                                        }
                                        room = replaceAt >= 0;
                                    }
                                    Vec3 position{};
                                    if (room && TransformPosition(node, position))
                                    {
                                        JointCandidate candidate;
                                        candidate.bone = bone;
                                        candidate.node = node;
                                        candidate.parent = nodeParent;
                                        candidate.rig = nodeRig;
                                        candidate.pos = position;
                                        candidate.isJnt = isJnt;
                                        std::snprintf(candidate.name, sizeof(candidate.name), "%s",
                                                      normalised);
                                        if (replaceAt >= 0)
                                        {
                                            candidates[replaceAt] = candidate;
                                        }
                                        else
                                        {
                                            ++candidatesPerBone[bone];
                                            candidates.push_back(candidate);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (pruned)
                {
                    continue;
                }

                const int children = UnboxInt(Invoke1(m_getChildCount, node, nullptr));
                for (int index = 0; index < children; ++index)
                {
                    if (pending.size() + visited >= kRigWalkMaxNodes)
                    {
                        break;
                    }
                    int childIndex = index;
                    g_stage = "rig: GetChild";
                    void *child = Invoke1(m_getChild, node, &childIndex);
                    if (child != nullptr && Alive(child))
                    {
                        pending.push_back(child);
                        pendingParent.push_back(node);
                        pendingRig.push_back(nodeRig);
                    }
                }
            }

            // Every joint of one mainRigBody is one body. When the strict table
            // resolved a full set, the figure binds to that single node and
            // cannot borrow a neighbour's joint; the hips decide WHICH body when
            // the walk saw more than one rig under the same root.
            bool exactAssigned = false;
            if (exactJoints.size() >= static_cast<size_t>(kHumanoidJointThreshold))
            {
                struct RigGroup
                {
                    void *rig = nullptr;
                    int bones = 0;
                    Vec3 hips{};
                    bool haveHips = false;
                    bool haveCrown = false;   // M_head_2_jnt found (male skin)
                    bool underSelf = false;   // rig hangs under slot.transform
                };
                std::vector<RigGroup> groups;
                groups.reserve(4);
                for (const JointCandidate &candidate : exactJoints)
                {
                    RigGroup *group = nullptr;
                    for (RigGroup &existing : groups)
                    {
                        if (existing.rig == candidate.rig)
                        {
                            group = &existing;
                            break;
                        }
                    }
                    if (group == nullptr)
                    {
                        groups.push_back(RigGroup{});
                        group = &groups.back();
                        group->rig = candidate.rig;
                    }
                    ++group->bones;
                    if (candidate.bone == BoneHips)
                    {
                        group->hips = candidate.pos;
                        group->haveHips = true;
                    }
                    // A male skin exposes both M_head_1_jnt (base) and
                    // M_head_2_jnt (crown). headIsBase used to stick at true
                    // from the first of them, so CrownFromBase() overwrote the
                    // real crown with a computed neck + 0.21 and the log called
                    // every 20-joint male rig "female skull base".
                    if (candidate.bone == BoneHead && !candidate.headIsBase)
                    {
                        group->haveCrown = true;
                    }
                }
                const bool freshFeet = slot.haveFeet && slot.feetTick != 0 &&
                                       GetTickCount64() - slot.feetTick <= 2000;
                RigGroup *best = nullptr;
                float bestScore = 1.0e9f;
                for (RigGroup &group : groups)
                {
                    if (group.bones < kHumanoidJointThreshold)
                    {
                        continue;
                    }
                    // Ownership first: a mainRigBody that hangs under this
                    // character's root is this character's, however far its hips
                    // sit from lastFeet. Distance is only the tie-breaker. The
                    // root is charRoot when the name walk found one: a view
                    // component is not always an ancestor of the rig (a body
                    // parented to a car seat, a skin whose mainRigBody is a
                    // sibling of the view), and testing against slot.transform
                    // alone threw the seated rig away.
                    void *ownerRoot = slot.charRoot != nullptr ? slot.charRoot : slot.transform;
                    group.underSelf =
                        group.rig != nullptr && RigUnderTransform(ownerRoot, group.rig);
                    float score = 0.0f;
                    if (freshFeet && group.haveHips)
                    {
                        const float dx = group.hips.x - slot.lastFeet.x;
                        const float dz = group.hips.z - slot.lastFeet.z;
                        score = std::sqrt(dx * dx + dz * dz);
                    }
                    const bool better =
                        (best == nullptr) ||
                        (group.underSelf != best->underSelf ? group.underSelf
                                                            : score < bestScore);
                    if (better)
                    {
                        best = &group;
                        bestScore = score;
                    }
                }
                // Proven ownership, no fresh feet to compare against, or hips
                // within 2.5 m: the seated player's own rig must not be thrown
                // away just because the animation moved the hips off the anchor.
                if (best != nullptr && (best->underSelf || !freshFeet || bestScore <= 2.5f))
                {
                    for (int bone = 0; bone < kBoneCount; ++bone)
                    {
                        slot.bone[bone] = nullptr;
                        slot.boneHold[bone] = 0;
                    }
                    for (const JointCandidate &candidate : exactJoints)
                    {
                        if (candidate.rig != best->rig)
                        {
                            continue;
                        }
                        // On a male skin (haveCrown) the base node must not
                        // overwrite M_head_2_jnt.
                        if (candidate.bone == BoneHead && candidate.headIsBase &&
                            best->haveCrown)
                        {
                            continue;
                        }
                        slot.bone[candidate.bone] = candidate.node;
                    }
                    slot.rigRoot = best->rig;
                    slot.headIsBase = !best->haveCrown;
                    exactAssigned = true;
                    char line[160];
                    std::snprintf(line, sizeof(line),
                                  "rig: exact table, %d joints under one mainRigBody "
                                  "(%zu rigs seen)%s",
                                  best->bones, groups.size(),
                                  slot.headIsBase ? ", female skull base" : "");
                    Trace(line);
                }
            }
            if (!exactAssigned)
            {
                // This rig names every skeleton node "..._jnt" and hangs LOD meshes,
                // eye helpers and hit-box placeholders under the same character. A
                // node called "..._head01_bald_lods" sits at the feet and used to win
                // the head slot, which drew the figure upside down; when the rig
                // clearly speaks the convention, only joints are considered.
                int jntCandidates = 0;
                for (const JointCandidate &candidate : candidates)
                {
                    if (candidate.isJnt)
                    {
                        ++jntCandidates;
                    }
                }
                if (jntCandidates >= kJntRigMinimum)
                {
                    candidates.erase(
                        std::remove_if(candidates.begin(), candidates.end(),
                                       [](const JointCandidate &candidate) {
                                           return !candidate.isJnt;
                                       }),
                        candidates.end());
                }

                PickJoints(slot, candidates);
            }

            int hits = 0;
            for (int bone = 0; bone < kBoneCount; ++bone)
            {
                if (slot.bone[bone] != nullptr)
                {
                    ++hits;
                }
            }
            // The walk landed on a leaf or on the wrong node: a body seated in a
            // vehicle is the normal case. Keep the rig that already read fine
            // instead of publishing an empty one.
            if (hits < kHumanoidJointThreshold && restoreKept())
            {
                return;
            }
            slot.boneHits = hits;
            // A rig that answered almost nothing is usually a half-loaded
            // character: retry later instead of drawing a limbless box for the
            // rest of the match. While the component is alive the retry is fast
            // - the old unconditional 4 s overrode the 500 ms PickJoints had
            // just set, which is the 4-second skeleton outage on seating.
            if (hits < kHumanoidJointThreshold)
            {
                slot.bonesResolved = false;
                slot.rigCooldown =
                    GetTickCount64() + (Alive(slot.transform) ? 500ull : kRigRetryMs);
            }
            else
            {
                slot.bonesResolved = true;
                // A resolved rig clears the fault ladder: the next failure on
                // this slot starts again at one second, not at five.
                slot.rigFaults = 0;
                // Fresh node set: this pass establishes where these joints sit
                // relative to the body, the next pass may judge them.
                slot.boneBaseline = true;
                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    slot.boneHold[bone] = 0;
                }
            }

            char text[160];
            std::snprintf(text, sizeof(text), "rig: %d/%d joints by position (%zu nodes, %zu cand)",
                          hits, kBoneCount, visited, candidates.size());
            Trace(text);

            // One-shot dump of the whole node list of the first character: the
            // joint table has to be written against where this rig actually puts
            // its nodes, and a name alone has now been wrong twice. Written even
            // when nothing matched, so a rig whose names are all foreign still
            // leaves evidence behind.
            if (!g_rigNamesDumped && !collected.empty())
            {
                g_rigNamesDumped = true;
                float low = candidates.empty() ? 0.0f : candidates[0].pos.y;
                for (const JointCandidate &candidate : candidates)
                {
                    low = std::min(low, candidate.pos.y);
                }
                std::string body;
                char header[192];
                std::snprintf(header, sizeof(header),
                              "character=%s nodes=%zu matched=%d/%d animator=%d\n"
                              "groundY=%.2f  (dy below = height above ground)\n\n",
                              slot.name, visited, hits, kBoneCount, slot.animator != nullptr ? 1 : 0,
                              low);
                body += header;
                body += collected;
                if (!candidates.empty())
                {
                    body += "\n-- chosen joints --\n";
                    float middleX = 0.0f;
                    float middleZ = 0.0f;
                    for (const JointCandidate &candidate : candidates)
                    {
                        middleX += candidate.pos.x;
                        middleZ += candidate.pos.z;
                    }
                    middleX /= static_cast<float>(candidates.size());
                    middleZ /= static_cast<float>(candidates.size());
                    for (const JointCandidate &candidate : candidates)
                    {
                        char line[160];
                        std::snprintf(line, sizeof(line),
                                      "bone=%2d  %-30s  dx=%6.2f dy=%6.2f dz=%6.2f\n",
                                      candidate.bone, candidate.name,
                                      candidate.pos.x - middleX, candidate.pos.y - low,
                                      candidate.pos.z - middleZ);
                        body += line;
                    }
                }
                // What PickJoints ACTUALLY assigned, one line per bone. The
                // section above lists MatchBone results per node, which is a
                // different thing and has already been misread once: it says
                // nothing about which node ended up drawn.
                body += "\n-- final slots --\n";
                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    const char *name = "(none)";
                    for (const JointCandidate &candidate : candidates)
                    {
                        if (candidate.node == slot.bone[bone])
                        {
                            name = candidate.name;
                            break;
                        }
                    }
                    char line[96];
                    std::snprintf(line, sizeof(line), "slot=%2d  %s\n", bone, name);
                    body += line;
                }
                WriteDebugFile(L"OmegaCore_rig.txt", body);
            }
            // Independent of the rig dump: the calibration only needs the
            // candidates PickJoints already produced.
            if (!g_nativeProbeDone)
            {
                g_nativeProbeDone = true;
                ProbeNativeJoints(candidates);
            }
        }

        // Resolves one character's skeleton.
        //
        // The name walk runs FIRST and needs no Animator at all. It used to run
        // second, behind an Animator.get_isHuman gate, and that invoke was the
        // single address that raised on every character: thirty caught faults in
        // thirteen seconds of a live session, each one parking a character for
        // thirty seconds, which is exactly why the skeleton disappeared and why
        // the walk itself only advanced once per backoff (~13 s).
        //
        // The Animator is only a bonus now, and only when the hierarchy said
        // nothing: GetBoneTransform on a non-humanoid avatar throws, and a throw
        // is charged as a fault, so it may never be the first thing tried.
        // Declared here because ResolveRig verifies the rig it just walked
        // actually hangs under this slot's own transform.
        bool RigUnderTransform(void *transform, void *rigRoot);

        void ResolveRig(Slot &slot)
        {
            // The rig is the one optional, expensive, per-character stage, so it
            // runs soft: a caught fault inside the hierarchy walk costs this
            // character's skeleton and nothing else. Before this, one dead
            // Animator or one destroyed node stood the whole ESP down for up to
            // 12.8 s — that is the "updates every 13 seconds" Olivia saw.
            g_softInvoke = true;
            g_softFaulted = false;
            ResolveRigByNames(slot);
            // The name walk climbs while the chain is single-child to find the
            // model root, and that climb can leave the character entirely: it
            // lands on a shared container, the DFS then finds the neighbour's
            // mainRigBody first, and the joints come back as a whole limb set
            // sitting about a metre from this body's feet. Measured live:
            // travelled=0.00 m, 16 joints moved 1.00 m relative to the anchor.
            // The scan already trusts this invariant, so it is checked here too.
            if (slot.rigRoot != nullptr && slot.boneHits >= kHumanoidJointThreshold &&
                !RigOwnedBySlot(slot))
            {
                char line[128];
                std::snprintf(line, sizeof(line),
                              "rig: rejected, mainRigBody does not hang under this body");
                Trace(line);
                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    slot.bone[bone] = nullptr;
                    slot.boneOk[bone] = false;
                }
                slot.boneHits = 0;
                slot.rigRoot = nullptr;
                slot.headIsBase = false;
                slot.bonesResolved = false;
            }
            const bool faulted = g_softFaulted;
            g_softInvoke = false;
            if (faulted)
            {
                // Do not retry a rig that raised: the same pointer raises again.
                // The wait grows 1 s -> 2 s -> 5 s instead of parking the body
                // for a full minute. A player who stepped out of a car used to
                // sit boneless until the next respawn, because one soft fault
                // bought a 60 s cooldown on a body that was perfectly fine.
                slot.bonesResolved = false;
                slot.boneHits = 0;
                const unsigned long long wait = slot.rigFaults == 0   ? 1000ull
                                                : slot.rigFaults == 1 ? 2000ull
                                                                      : 5000ull;
                if (slot.rigFaults < 8)
                {
                    ++slot.rigFaults;
                }
                slot.rigCooldown = GetTickCount64() + wait;
                char line[96];
                std::snprintf(line, sizeof(line), "rig: parked after a fault (%llu ms)",
                              static_cast<unsigned long long>(wait));
                Trace(line);
            }
        }

        // Whether a body can land on screen at all this frame. Joint reads are
        // the most expensive thing the overlay does - 19 managed
        // Transform.get_position calls per body, each one boxing a Vector3 - and
        // they were paid for every slot in the list, including the ones behind
        // the camera and the ones on the other side of the map. The body is
        // tested as a feet->crown segment projected with the matrix already in
        // hand; the margin covers a rig that leans or reaches outside its own
        // box, so a character just off the edge still gets read one frame early
        // instead of popping in bare.
        bool SlotNearViewport(const Slot &slot, float displayWidth, float displayHeight)
        {
            if (!slot.haveFeet)
            {
                // Nothing read yet - let it through, the read is what fills the
                // position in the first place.
                return true;
            }
            constexpr float kBodyHeight = 2.0f;
            constexpr float kMarginPixels = 240.0f;
            const float minX = -kMarginPixels;
            const float minY = -kMarginPixels;
            const float maxX = displayWidth + kMarginPixels;
            const float maxY = displayHeight + kMarginPixels;
            float feetX = 0.0f;
            float feetY = 0.0f;
            float crownX = 0.0f;
            float crownY = 0.0f;
            float depth = 0.0f;
            const Vec3 crown = Vec3{ slot.lastFeet.x, slot.lastFeet.y + kBodyHeight, slot.lastFeet.z };
            const bool feetOk = Project(slot.lastFeet, displayWidth, displayHeight, feetX, feetY, depth);
            const bool crownOk = Project(crown, displayWidth, displayHeight, crownX, crownY, depth);
            if (!feetOk && !crownOk)
            {
                // One of the two ends is enough to decide; both failing means the
                // body is behind the camera or past the far plane, where the game
                // does not draw it either.
                return false;
            }
            const float left = feetOk && crownOk ? std::min(feetX, crownX) : (feetOk ? feetX : crownX);
            const float right = feetOk && crownOk ? std::max(feetX, crownX) : (feetOk ? feetX : crownX);
            const float top = feetOk && crownOk ? std::min(feetY, crownY) : (feetOk ? feetY : crownY);
            const float bottom = feetOk && crownOk ? std::max(feetY, crownY) : (feetOk ? feetY : crownY);
            return right >= minX && left <= maxX && bottom >= minY && top <= maxY;
        }

        // The two budgeted skeleton stages, called every tick:
        //   * up to three rigs resolved per tick (19 GetBoneTransform calls, or a
        //     capped hierarchy read for a generic rig), inside a 3 ms budget;
        //   * pose reads for the bodies that are actually on screen.
        // Between ticks the pose is extrapolated by the projection, so the
        // skeleton follows a running character without a per-frame read.
        // Declared here because BonePass checks joint ownership and the walk
        // itself lives next to Scan() further down.
        bool RigUnderTransform(void *transform, void *rigRoot);

        void BonePass(float displayWidth, float displayHeight)
        {
            ++g_bonePasses;
            if (!g_options.skeleton || g_slots.empty())
            {
                return;
            }

            // The managed walk stays on even while poses land: the shared hitbox
            // buffer only carries seven torso points, so the 19 limb joints can
            // only come from the view slots. Returning early here is what made
            // every character a bare spine line.
            const size_t slots = g_slots.size();
            const ULONGLONG now = GetTickCount64();
            // Баг 1.6: startedUs перенесён НИЖЕ цикла ResolveRig().
            // Ранее он захватывал время до ResolveRig (3000–6000 us),
            // и бюджет 4000 us срабатывал после 1-го игрока — остальные
            // простаивали со старыми позами до 91 сек (boneAge в логе).
            // Теперь бюджет относится только к чтению поз (TransformPosition).
            //
            // One rig per tick was the old rule, and it is exactly the "bones
            // come back 5-60 seconds after leaving the car" report: a body that
            // had just been dropped queued behind every other unresolved
            // character, one per frame. Three per tick still fits a frame, and
            // the wall-clock budget cuts the pass short if the walk is slow.
            if (g_rigCursor >= slots)
            {
                g_rigCursor = 0;
            }
            // Slots whose rig was built in THIS tick. Their joints are read
            // further down in the same pass, so a character that just stepped
            // out of a car has a skeleton on the very next frame instead of
            // waiting for the bone cursor to come round to it.
            size_t freshRigs[3] = {0, 0, 0};
            int freshRigCount = 0;
            const ULONGLONG rigStartedUs = NowMicroseconds();
            for (size_t step = 0; step < slots; ++step)
            {
                if (freshRigCount >= 3 || NowMicroseconds() - rigStartedUs >= 3000)
                {
                    break;
                }
                const size_t index = g_rigCursor;
                g_rigCursor = (g_rigCursor + 1) % slots;
                Slot &slot = g_slots[index];
                if (slot.bonesResolved || slot.rigCooldown > now)
                {
                    continue;
                }
                // Resolved fresh on every attempt. A cached Animator wrapper
                // survives its native object being destroyed when a character is
                // pooled or respawned, and every invoke on that wrapper is an
                // access violation at one fixed address — the trace showed the
                // same rip 30 times in a row. Two managed calls once per slot per
                // rig pass is cheaper than a poisoned thread.
                slot.animator = ResolveAnimator(slot.component);
                slot.animatorResolved = true;
                if (slot.animator != nullptr && !Alive(slot.animator))
                {
                    slot.animator = nullptr;
                }
                const ULONGLONG faultMark = g_faultUntil;
                ResolveRig(slot);
                if (g_faultUntil != faultMark)
                {
                    // The rig of this character cannot be read at all (a stale
                    // animator, a destroyed node). Retrying it every tick is what
                    // turned one bad slot into a fault storm — 10 caught faults in
                    // 15 s of a live session, each one a stand-down for the whole
                    // ESP. The slot is parked, but only for three seconds: the old
                    // 30 s is what kept a body boneless long after it was fine.
                    slot.rigCooldown = now + 3000;
                    break;
                }
                if (slot.boneHits >= kHumanoidJointThreshold)
                {
                    freshRigs[freshRigCount++] = index;
                }
            }

            // Баг 1.6: бюджет отсчитывается только от начала чтения поз.
            const ULONGLONG startedUs = NowMicroseconds();

            // Only bodies that are really read count against the cap and the
            // budget. Counting loop iterations meant a list of mostly off-screen
            // bodies burned the whole allowance on skipped slots and the visible
            // ones got their joints late - the "bones lag then snap" report.
            int bodiesRead = 0;
            // Step 3 of the spec: order the pass by what the eye can see. With
            // 20+ players the old round-robin cursor spent the whole budget on
            // whoever came next in the slot list, so a target in the middle of
            // the screen got its joints once every 10-20 frames - boneAge in the
            // seconds and dm= of 0.7-1.16 m on a running body. Order is now:
            // rigs built in this tick, then every body inside the viewport
            // within kNearFirstMeters, nearest to the eye first, then the rest
            // round-robin so a distant group still gets refreshed eventually.
            constexpr float kNearFirstMeters = 150.0f;
            const size_t slotLimit = std::min(slots, kMaxEntities);
            bool taken[kMaxEntities] = {};
            bool eligible[kMaxEntities] = {};
            float slotDist[kMaxEntities] = {};
            for (size_t i = 0; i < slotLimit; ++i)
            {
                taken[i] = false;
                const Slot &slot = g_slots[i];
                const bool onScreen = SlotNearViewport(slot, displayWidth, displayHeight);
                eligible[i] = onScreen && slot.boneHits != 0;
                slotDist[i] = (!g_cameraPositionValid || !slot.haveFeet)
                                  ? 1.0e9f
                                  : aim::Length(aim::Sub(slot.lastFeet, g_cameraPosition));
                // A visible body with no joints is the exact state Olivia
                // reports as "no skeleton on this one". Rate limited, because a
                // body parked in that state would otherwise print every frame.
                static ULONGLONG visibleMissAt = 0;
                if (onScreen && slot.boneHits == 0 && now - visibleMissAt >= 5000)
                {
                    visibleMissAt = now;
                    char miss[192];
                    std::snprintf(miss, sizeof(miss),
                                  "rig_miss: visible slot has no joints (dist=%.0f m, "
                                  "resolved=%d, cd=%lld, anim=%d, root=%d)",
                                  slotDist[i] >= 1.0e8f ? -1.0f : slotDist[i],
                                  slot.bonesResolved ? 1 : 0,
                                  static_cast<long long>(slot.rigCooldown) -
                                      static_cast<long long>(now),
                                  slot.animator != nullptr ? 1 : 0,
                                  slot.rigRoot != nullptr ? 1 : 0);
                    TraceLine(miss);
                }
            }
            size_t order[kMaxEntities];
            int orderCount = 0;
            for (int f = 0; f < freshRigCount && orderCount < static_cast<int>(kMaxEntities); ++f)
            {
                const size_t index = freshRigs[f];
                if (index < slotLimit && !taken[index])
                {
                    taken[index] = true;
                    order[orderCount++] = index;
                }
            }
            for (;;)
            {
                size_t pick = kMaxEntities;
                float bestDist = 0.0f;
                for (size_t i = 0; i < slotLimit; ++i)
                {
                    if (taken[i] || !eligible[i] || slotDist[i] > kNearFirstMeters)
                    {
                        continue;
                    }
                    if (pick == kMaxEntities || slotDist[i] < bestDist)
                    {
                        pick = i;
                        bestDist = slotDist[i];
                    }
                }
                if (pick == kMaxEntities || orderCount >= static_cast<int>(kMaxEntities))
                {
                    break;
                }
                taken[pick] = true;
                order[orderCount++] = pick;
            }
            if (slotLimit > 0)
            {
                if (g_boneCursor >= slotLimit)
                {
                    g_boneCursor = 0;
                }
                for (size_t k = 0; k < slotLimit && orderCount < static_cast<int>(kMaxEntities); ++k)
                {
                    const size_t i = (g_boneCursor + k) % slotLimit;
                    if (taken[i])
                    {
                        continue;
                    }
                    taken[i] = true;
                    order[orderCount++] = i;
                }
                ++g_boneCursor;
            }
            for (int n = 0; n < orderCount && bodiesRead < kSlotsPerBonePass; ++n)
            {
                // Wall-clock budget in microseconds, not GetTickCount64: that
                // counter ticks in 15.6 ms steps, so the old test either cut the
                // pass after one body or let six (114 managed calls) through in
                // the same millisecond. 4000 us covers most of an 11-player pass
                // and still leaves the frame room to draw.
                if (bodiesRead > 0 && NowMicroseconds() - startedUs >= 4000)
                {
                    break;
                }
                const size_t index = order[n];
                // Bodies that cannot be drawn this frame are not read: the joint
                // pass used to spend the budget on characters behind the camera
                // or across the map. The decision was made once above, so the
                // skip costs no second projection.
                if (!eligible[index])
                {
                    continue;
                }
                Slot &slot = g_slots[index];
                ++bodiesRead;
                int failed = 0;
                int read = 0;
                // The anchor is read FIRST, in the same pass as the joints, so
                // bonePos[] and poseFeet describe one instant. Reading feet in
                // Sample() and joints a tick later put a moving body's joints
                // 10-30 cm away from its own anchor, which is a visible snap on
                // every capture.
                Vec3 captureFeet = slot.lastFeet;
                // The figure is drawn as `base + (joint - anchor)`, and the base
                // is the body's own liveliness read every sample. The anchor has
                // to be the SAME node, not a neighbouring one: the model root and
                // the view component are two different transforms, and their
                // difference was applied to all 19 joints as a constant shift
                // whose animation-to-animation variation reads as the skeleton
                // leading or trailing the body on a run. The rig root
                // (`mainRigBody`) is the node the joints actually hang from, so it
                // is the only exact anchor; the view transform is a fallback for a
                // body whose rig is not resolved yet.
                bool haveCaptureFeet = false;
                if (slot.rigRoot != nullptr && TransformPosition(slot.rigRoot, captureFeet))
                {
                    haveCaptureFeet = true;
                }
                else if (slot.transform != nullptr && TransformPosition(slot.transform, captureFeet))
                {
                    haveCaptureFeet = true;
                }
                else if (slot.haveFeet)
                {
                    captureFeet = slot.lastFeet;
                    haveCaptureFeet = true;
                }
                const Vec3 previousAnchor = slot.poseFeet;
                Vec3 previousPosition[kBoneCount];
                bool previousOk[kBoneCount];
                // The joints as read in THIS pass, kept untouched: the hold logic
                // below rewrites slot.bonePos[] in place, and the diagnostic that
                // runs afterwards has to see what the pass actually measured.
                Vec3 newPosition[kBoneCount];
                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    previousOk[bone] = slot.boneOk[bone];
                    previousPosition[bone] = slot.bonePos[bone];
                }
                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    if (slot.bone[bone] == nullptr)
                    {
                        slot.boneOk[bone] = false;
                        continue;
                    }
                    ++read;
                    slot.boneOk[bone] = TransformPosition(slot.bone[bone], slot.bonePos[bone]);
                    newPosition[bone] = slot.bonePos[bone];
                    if (!slot.boneOk[bone])
                    {
                        ++failed;
                    }
                }
                // A joint that moved further, RELATIVE TO THE BODY'S OWN FEET,
                // than any limb can travel between two passes has changed owner.
                // The old check measured world distance, so a body riding a car
                // (or a respawn) tripped it while a neighbour's arm - the body
                // itself stationary - did not, which is why the 96 m and 100 m
                // lines in the log never dropped anything. Relative movement is
                // the only measure that separates "the body moved" from "this
                // joint now belongs to somebody else".
                int jumped = 0;
                int held = 0;
                int wrongOwner = 0;
                int firstJump = -1;
                float firstJumpLength = 0.0f;
                const bool baselinePass = slot.boneBaseline;
                slot.boneBaseline = false;
                // Detect a coherent whole-rig shift (seating in a vehicle,
                // respawn, rigRoot swap): when at least 3 core bones move by
                // the same vector (> 0.85 m but spread < 0.25 m), accept the
                // new pose as a new baseline and clear boneHold so the jump
                // filter does not suppress it. Without this, entering a car
                // produced 10+ "held back" lines while the freeze lasted.
                bool wholeRigShift = false;
                if (!baselinePass && read > 0 && haveCaptureFeet)
                {
                    const int coreBones[4] = { BoneHips, BoneSpine, BoneChest, BoneHead };
                    Vec3 coreDeltas[4]{};
                    int shiftedCore = 0;
                    for (int k = 0; k < 4; ++k)
                    {
                        const int b = coreBones[k];
                        if (!slot.boneOk[b] || !previousOk[b])
                        {
                            continue;
                        }
                        const Vec3 before = aim::Sub(previousPosition[b], previousAnchor);
                        const Vec3 after  = aim::Sub(newPosition[b],      captureFeet);
                        const Vec3 delta  = aim::Sub(after, before);
                        const float jumpLen = aim::Length(delta);
                        if (aim::Finite(jumpLen) && jumpLen > 0.85f)
                        {
                            coreDeltas[shiftedCore++] = delta;
                        }
                    }
                    if (shiftedCore >= 3)
                    {
                        wholeRigShift = true;
                        for (int k = 1; k < shiftedCore; ++k)
                        {
                            if (aim::Length(aim::Sub(coreDeltas[k], coreDeltas[0])) > 0.25f)
                            {
                                wholeRigShift = false;
                                break;
                            }
                        }
                    }
                    if (wholeRigShift)
                    {
                        for (int bone = 0; bone < kBoneCount; ++bone)
                        {
                            slot.boneHold[bone] = 0;
                        }
                    }
                }
                if (!baselinePass && !wholeRigShift && read > 0 && haveCaptureFeet)
                {
                    // Core and legs keep the old 0.85 m rule: a body changes
                    // place as a whole (seating, respawn) and must not be
                    // mistaken for a borrowed joint.
                    for (int bone = 0; bone <= BoneHead; ++bone)
                    {
                        if (!slot.boneOk[bone] || !previousOk[bone])
                        {
                            slot.boneHold[bone] = 0;
                            continue;
                        }
                        const Vec3 before = aim::Sub(previousPosition[bone], previousAnchor);
                        const Vec3 after = aim::Sub(newPosition[bone], captureFeet);
                        const float jump = aim::Length(aim::Sub(after, before));
                        if (!aim::Finite(jump) || jump <= 0.85f)
                        {
                            slot.boneHold[bone] = 0;
                            continue;
                        }
                        ++jumped;
                        if (firstJump < 0 || jump > firstJumpLength)
                        {
                            firstJump = bone;
                            firstJumpLength = jump;
                        }
                        if (slot.boneHold[bone] == 0)
                        {
                            // First read of the spike. Put the joint back where
                            // this body last had it instead of dropping it for
                            // a pass: dropping made the arm vanish and come
                            // back one pass later already attached to the
                            // neighbour, which is the visible second of
                            // "someone else's bones". Visible at the old place,
                            // waiting for a second read, is the honest state.
                            slot.boneHold[bone] = 1;
                            // Roll back the OFFSET, not the old world point:
                            // poseFeet is rewritten to captureFeet at the end of
                            // this pass, so a world-space rollback leaves the
                            // joint `travelled` metres behind a moving body
                            // (measured 1.80 m on a car).
                            slot.bonePos[bone] = aim::Add(captureFeet, before);
                            ++held;
                        }
                        else
                        {
                            // The joint has been in the same new place on two
                            // passes. That is only believable while the node is
                            // still this body's: a pooled rig hands the same
                            // Transform pointers to the next character, and then
                            // the "new place" is a neighbour's arm. The parent
                            // chain answers that without a managed name read, and
                            // it runs only on a jump, so the 19-joint pass pays
                            // nothing in the normal case.
                            if (slot.rigRoot != nullptr &&
                                !RigUnderTransform(slot.rigRoot, slot.bone[bone]))
                            {
                                ++wrongOwner;
                                slot.boneHold[bone] = 0;
                                slot.bonePos[bone] = previousPosition[bone];
                                slot.boneOk[bone] = previousOk[bone];
                                continue;
                            }
                            slot.boneHold[bone] = 0;
                        }
                    }
                    // Arms are one limb, not four independent joints, but the
                    // two-pass hold is gone. That hold was the flip-flop: a
                    // burst of 2-4 pre-IK captures overflows the counter, pre-IK
                    // becomes the norm, and the next post-IK captures get rolled
                    // back - the fixed-step 0.33-0.49 m jump measured on
                    // standing armed bodies. What is left is the two states that
                    // are not a pose at all: a chain torn in half (a segment
                    // over 0.45 m means shoulder pre-IK and hand post-IK) and a
                    // joint that no longer hangs under this body's rig root (a
                    // pooled Transform handed to a neighbour).
                    const int arms[2][4] = {
                        { BoneLeftShoulder, BoneLeftUpperArm, BoneLeftLowerArm, BoneLeftHand },
                        { BoneRightShoulder, BoneRightUpperArm, BoneRightLowerArm, BoneRightHand }
                    };
                    for (int s = 0; s < 2; ++s)
                    {
                        const int sh = arms[s][0];
                        const int ua = arms[s][1];
                        const int la = arms[s][2];
                        const int ha = arms[s][3];
                        bool torn = false;
                        // Per-segment limits, not one number for the whole arm:
                        // shoulder->upperarm is a short clavicle span, the two
                        // long bones are 0.55 m at most even on a tall skin with
                        // the arm raised on a weapon. A single 0.45 m threshold
                        // on the lower bones called a normal aiming arm "torn"
                        // and rolled it back every pass.
                        if (slot.boneOk[sh] && slot.boneOk[ua] &&
                            aim::Length(aim::Sub(newPosition[ua], newPosition[sh])) > 0.45f)
                        {
                            torn = true;
                        }
                        if (slot.boneOk[ua] && slot.boneOk[la] &&
                            aim::Length(aim::Sub(newPosition[la], newPosition[ua])) > 0.55f)
                        {
                            torn = true;
                        }
                        if (slot.boneOk[la] && slot.boneOk[ha] &&
                            aim::Length(aim::Sub(newPosition[ha], newPosition[la])) > 0.55f)
                        {
                            torn = true;
                        }
                        bool foreign = false;
                        float worstJump = 0.0f;
                        int worstBone = -1;
                        for (int k = 0; k < 4; ++k)
                        {
                            const int b = arms[s][k];
                            if (!slot.boneOk[b] || !previousOk[b])
                            {
                                continue;
                            }
                            const Vec3 before = aim::Sub(previousPosition[b], previousAnchor);
                            const Vec3 after = aim::Sub(newPosition[b], captureFeet);
                            const float jump = aim::Length(aim::Sub(after, before));
                            if (!aim::Finite(jump))
                            {
                                continue;
                            }
                            if (jump > worstJump)
                            {
                                worstJump = jump;
                                worstBone = b;
                            }
                            if (jump > 0.85f && slot.rigRoot != nullptr &&
                                slot.bone[b] != nullptr &&
                                !RigUnderTransform(slot.rigRoot, slot.bone[b]))
                            {
                                foreign = true;
                            }
                        }
                        if (foreign)
                        {
                            ++wrongOwner;
                        }
                        if (worstBone >= 0 && worstJump > 0.85f)
                        {
                            ++jumped;
                            if (firstJump < 0 || worstJump > firstJumpLength)
                            {
                                firstJump = worstBone;
                                firstJumpLength = worstJump;
                            }
                        }
                        if (!torn && !foreign)
                        {
                            continue;
                        }
                        ++held;
                        for (int k = 0; k < 4; ++k)
                        {
                            const int b = arms[s][k];
                            if (!previousOk[b])
                            {
                                // The joint was not readable last pass either:
                                // nothing to roll back to, so the raw read
                                // stands instead of the hand blinking out.
                                continue;
                            }
                            const Vec3 before = aim::Sub(previousPosition[b], previousAnchor);
                            slot.bonePos[b] = aim::Add(captureFeet, before);
                            slot.boneOk[b] = true;
                        }
                    }
                }
                if (jumped > 0)
                {
                    static ULONGLONG jumpTraceAt = 0;
                    const ULONGLONG jumpNow = GetTickCount64();
                    if (jumpNow - jumpTraceAt >= 1000)
                    {
                        jumpTraceAt = jumpNow;
                        char line[176];
                        std::snprintf(line, sizeof(line),
                                      "bone jump: %d joint(s), worst index=%d %.2f m relative, "
                                      "%d held back, %d of %d read",
                                      jumped, firstJump, firstJumpLength, held,
                                      read - failed, read);
                        TraceLine(line);
                        // Which joints moved, and how far each one is from the
                        // anchor now. A whole-rig offset means the anchor is
                        // stale; a limbs-only offset means the limbs were
                        // re-read against a different body's feet.
                        char detail[256];
                        int okCount = 0;
                        for (int bone = 0; bone < kBoneCount; ++bone)
                        {
                            if (slot.boneOk[bone])
                            {
                                ++okCount;
                            }
                        }
                        int written = std::snprintf(
                            detail, sizeof(detail),
                            "bone jump ctx: travelled=%.2f ok=%d hips=%.1f,%.1f,%.1f feet=%.1f,%.1f,%.1f",
                            aim::Length(aim::Sub(captureFeet, previousAnchor)), okCount,
                            newPosition[BoneHips].x, newPosition[BoneHips].y, newPosition[BoneHips].z,
                            captureFeet.x, captureFeet.y, captureFeet.z);
                        for (int bone = 0; bone < kBoneCount && written > 0 &&
                                            written < static_cast<int>(sizeof(detail)) - 24; ++bone)
                        {
                            if (!slot.boneOk[bone] || !previousOk[bone])
                            {
                                continue;
                            }
                            const Vec3 before = aim::Sub(previousPosition[bone], previousAnchor);
                            const Vec3 after = aim::Sub(newPosition[bone], captureFeet);
                            const float jump = aim::Length(aim::Sub(after, before));
                            if (jump <= 0.85f)
                            {
                                continue;
                            }
                            const float fromFeet =
                                aim::Length(aim::Sub(newPosition[bone], captureFeet));
                            written += std::snprintf(detail + written, sizeof(detail) - written,
                                                     " %d:%.2f/%.2f", bone, jump, fromFeet);
                        }
                        TraceLine(detail);
                    }
                }
                // A joint that moved but does not hang under this slot's
                // mainRigBody is somebody else's joint. Waiting a second pass
                // would draw the neighbour's arm for a whole capture, which is
                // the visible "bones of another player for a second" report.
                if (wrongOwner > 0)
                {
                    slot.bonesResolved = false;
                    slot.boneHits = 0;
                    slot.rigRoot = nullptr;
                    slot.headIsBase = false;
                    slot.rigCooldown = now + 500;
                    slot.poseFeet = captureFeet;
                    slot.lastFeet = captureFeet;
                    slot.haveFeet = haveCaptureFeet;
                    slot.poseTick = NowMicroseconds();
                    char line[128];
                    std::snprintf(line, sizeof(line),
                                  "rig: dropped, %d joint(s) left this mainRigBody",
                                  wrongOwner);
                    TraceLine(line);
                    continue;
                }
                // A body does not travel 15 m between two joint passes. If the
                // feet did, this slot now carries a different character (the view
                // component came back from the pool under a new model), and the
                // joints cached for the previous one are worthless. This runs
                // BEFORE the four-joint rule: a teleport moves every joint, so
                // the four-joint path would claim it first and park the slot for
                // the full 4 s retry instead of the fast one.
                //
                // Three things made this rule fire on healthy bodies:
                //  - a fresh slot has poseTick == 0 and poseFeet == {0,0,0}, so
                //    the first pass measured the distance from the map centre
                //    (43..51 m in the log) and dropped the rig it had just built;
                //  - a car that left the viewport for a second keeps its old
                //    poseFeet, so coming back read as a teleport;
                //  - seating and unseating move the body a couple of metres.
                // So: skip the first pass, scale the limit by elapsed time, and
                // accept the new pose whenever the rig still hangs under this
                // slot's transform - ownership, not distance, decides.
                if (haveCaptureFeet && slot.haveFeet && slot.poseTick != 0)
                {
                    const ULONGLONG nowUs = NowMicroseconds();
                    const float dtSec = (nowUs > slot.poseTick)
                                            ? static_cast<float>(nowUs - slot.poseTick) * 1e-6f
                                            : 0.0f;
                    const float maxMove = std::max(15.0f, 45.0f * dtSec);
                    const float travelled = aim::Length(aim::Sub(captureFeet, previousAnchor));
                    if (aim::Finite(travelled) && travelled > maxMove)
                    {
                        if (slot.rigRoot != nullptr && RigOwnedBySlot(slot))
                        {
                            // The body rode off screen or slid into a seat: the
                            // joints are still this character's, so take the
                            // fresh coordinates and keep the rig.
                            for (int bone = 0; bone < kBoneCount; ++bone)
                            {
                                slot.bonePos[bone] = newPosition[bone];
                                slot.boneHold[bone] = 0;
                            }
                            jumped = 0;
                        }
                        else
                        {
                            slot.bonesResolved = false;
                            slot.boneHits = 0;
                            slot.rigRoot = nullptr;
                            slot.headIsBase = false;
                            slot.rigCooldown = now + 500;
                            slot.poseFeet = captureFeet;
                            slot.lastFeet = captureFeet;
                            slot.poseTick = NowMicroseconds();
                            TraceLine("rig: dropped, body teleported and left transform");
                            continue;
                        }
                    }
                }
                // Four joints changing owner in one pass is not a bad joint, it
                // is the wrong body - UNLESS the rig still hangs under this
                // slot's transform. Seating and unseating move the hips, both
                // legs and the torso by 0.85-1.5 m in a single tick while the
                // root itself has not moved at all (travelled=0.00 in the log),
                // so the ownership check, not the jump count, decides.
                if (jumped >= 4)
                {
                    // Step 2: `slot.rigRoot != nullptr` was part of the test,
                    // and a rig that is still under its owner but whose root
                    // pointer was cleared (a rescan, a pooled component) was
                    // torn down on every seating. Geometry is stronger evidence:
                    // a hips->chest span of at most 0.70 m and a chest->neck span
                    // of at most 0.45 m cannot come from two bodies.
                    bool stillMine = true;
                    if (slot.rigRoot != nullptr && RigOwnedBySlot(slot) &&
                        slot.bone[BoneHips] != nullptr &&
                        !RigUnderTransform(slot.rigRoot, slot.bone[BoneHips]))
                    {
                        stillMine = false;  // the rig left this body's root
                    }
                    if (stillMine && slot.boneOk[BoneHips] && slot.bone[BoneHips] != nullptr &&
                        !Alive(slot.bone[BoneHips]))
                    {
                        stillMine = false;  // the joint object itself is gone
                    }
                    if (stillMine && slot.boneOk[BoneHips] && slot.boneOk[BoneChest] &&
                        aim::Length(aim::Sub(newPosition[BoneChest], newPosition[BoneHips])) > 0.70f)
                    {
                        stillMine = false;  // two torsos in one read
                    }
                    if (stillMine && slot.boneOk[BoneChest] && slot.boneOk[BoneNeck] &&
                        aim::Length(aim::Sub(newPosition[BoneNeck], newPosition[BoneChest])) >
                            0.45f)
                    {
                        stillMine = false;  // neck belongs to the neighbour
                    }
                    if (stillMine)
                    {
                        // Sitting down, standing up, a knockdown or a stance
                        // change: every joint is still this character's, so take
                        // the new pose immediately instead of tearing the rig
                        // down and waiting for a re-resolve.
                        for (int bone = 0; bone < kBoneCount; ++bone)
                        {
                            slot.bonePos[bone] = newPosition[bone];
                            slot.boneHold[bone] = 0;
                        }
                    }
                    else
                    {
                        slot.bonesResolved = false;
                        slot.boneHits = 0;
                        slot.rigRoot = nullptr;
                        slot.headIsBase = false;
                        slot.rigCooldown = now + 500;
                        slot.poseFeet = captureFeet;
                        slot.lastFeet = captureFeet;
                        slot.haveFeet = haveCaptureFeet;
                        slot.poseTick = NowMicroseconds();
                        TraceLine("rig: dropped, four or more joints changed owner");
                        continue;
                    }
                }
                // Identity check. The rig cached for this slot is a set of raw
                // Transform pointers kept for the session; the view component it
                // was resolved from is pooled, and the same component pointer
                // comes back with a different character under it, so the cached
                // pointers can belong to another body - the skeleton then draws
                // a neighbour's arm. Every joint of a body sits inside a small
                // envelope around that body's own centre: the spine chain within
                // 1.05 m, the limbs within 1.40 m. Four joints outside is not a
                // stretched pose, it is a different body.
                //
                // The centre is the hips, not the feet. Two reasons: the old
                // `bone <= BoneHead` test put the legs (1..6, which are below
                // BoneHead = 10 in the enum) in the TIGHT 0.80 m class, and a
                // seated pose stretches the knees and feet forward of the feet
                // anchor, so both together dropped every rig in a car.
                if (read > 0 && failed < read)
                {
                    Vec3 centre{};
                    bool haveCentre = false;
                    if (slot.boneOk[BoneHips])       { centre = slot.bonePos[BoneHips];  haveCentre = true; }
                    else if (slot.boneOk[BoneChest]) { centre = slot.bonePos[BoneChest]; haveCentre = true; }
                    // No `feet + 0.95` stand-in. On a body seated in a car the
                    // real torso sits where a standing hips would be, so the
                    // invented centre rejected every joint of the seated pose —
                    // the skeleton that vanished on seating. Without a measured
                    // torso joint there is no envelope to check against, and the
                    // cheaper checks above already ran.

                    if (haveCentre)
                    {
                        int farFromCentre = 0;
                        for (int bone = 0; bone < kBoneCount; ++bone)
                        {
                            if (!slot.boneOk[bone])
                            {
                                continue;
                            }
                            const Vec3 d = aim::Sub(slot.bonePos[bone], centre);
                            const bool isCore =
                                bone == BoneHips || (bone >= BoneSpine && bone <= BoneHead);
                            const float maxR = isCore ? 1.05f : 1.40f;
                            if (d.x * d.x + d.y * d.y + d.z * d.z > maxR * maxR)
                            {
                                ++farFromCentre;
                            }
                        }
                        // A hips joint 3.5 m away from the body's own feet is not
                        // a pose either - but only when the rig is no longer
                        // owned by this transform.
                        const bool rootDetached =
                            haveCaptureFeet && slot.boneOk[BoneHips] &&
                            aim::Length(aim::Sub(slot.bonePos[BoneHips], captureFeet)) > 3.5f &&
                            !(slot.rigRoot != nullptr && RigOwnedBySlot(slot));

                        if (farFromCentre >= 4 || rootDetached)
                        {
                            slot.bonesResolved = false;
                            slot.boneHits = 0;
                            slot.rigRoot = nullptr;
                            slot.headIsBase = false;
                            // Fast retry: a character walking back into view must
                            // not wait the full 4 s for its skeleton.
                            slot.rigCooldown = now + 500;
                            char line[112];
                            std::snprintf(line, sizeof(line),
                                          "rig: dropped, %d joints outside the body envelope",
                                          farFromCentre);
                            TraceLine(line);
                            continue;
                        }
                    }
                }
                // Баг 1.2: расширенная посегментная проверка конечностей.
                // Выбираем опорную точку: Chest → Spine → Hips → ноги+1.25 м.
                // Радиус 1.15 м, не 0.95: рука, поднятая на оружие
                // (PlayerWeaponAnimationView), уносит кисть на 1.0-1.1 м от
                // груди, и при 0.95 м кисть гасилась в boneOk=false каждый
                // проход — в логе это `joints=17/18` и прыжок кисти на 0.97 м.
                // Добавлена цепная проверка длин сегментов рук
                // (UpperArm→LowerArm→Hand ≤ 0.50 м).
                {
                    Vec3 torso{};
                    bool haveTorso = false;
                    if (slot.boneOk[BoneChest])      { torso = slot.bonePos[BoneChest]; haveTorso = true; }
                    else if (slot.boneOk[BoneSpine]) { torso = slot.bonePos[BoneSpine]; haveTorso = true; }
                    else if (slot.boneOk[BoneHips])  { torso = slot.bonePos[BoneHips];  haveTorso = true; }
                    // No `feet + 1.25` stand-in: a seated torso is not 1.25 m
                    // above the ground under the seat, and the invented point
                    // rejected every arm of a driver. No measured torso joint
                    // means no reference, so this check is skipped.

                    if (haveTorso)
                    {
                        constexpr float kMaxLimbFromTorsoSq = 1.15f * 1.15f;
                        for (int bone = BoneLeftShoulder; bone <= BoneRightHand; ++bone)
                        {
                            if (!slot.boneOk[bone]) continue;
                            const Vec3 d = aim::Sub(slot.bonePos[bone], torso);
                            if (d.x * d.x + d.y * d.y + d.z * d.z > kMaxLimbFromTorsoSq)
                                slot.boneOk[bone] = false;
                        }
                    }

                    // The torso is one rigid chain. A chest-to-neck span of more
                    // than 0.45 m or a hips-to-chest span of more than 0.70 m is
                    // not a pose — the rig has picked up a joint from the body
                    // standing next to it.
                    bool torsoStretched = false;
                    if (slot.boneOk[BoneChest] && slot.boneOk[BoneNeck] &&
                        aim::Length(aim::Sub(slot.bonePos[BoneNeck], slot.bonePos[BoneChest])) > 0.45f)
                    {
                        torsoStretched = true;
                    }
                    if (slot.boneOk[BoneHips] && slot.boneOk[BoneChest] &&
                        aim::Length(aim::Sub(slot.bonePos[BoneChest], slot.bonePos[BoneHips])) > 0.70f)
                    {
                        torsoStretched = true;
                    }
                    if (torsoStretched)
                    {
                        // One stretched read is a frame boundary, not a stranger.
                        // A car at 30 m/s crosses 0.4-0.56 m inside a single Unity
                        // physics tick, so the chest and the hips of the same body
                        // can come from two different moments. Tearing the rig
                        // down here is what left a player boneless for a second
                        // every time the vehicle moved; three passes in a row is
                        // a rig that really belongs to someone else.
                        ++slot.stretchStreak;
                        if (slot.stretchStreak >= 3 || !Alive(slot.rigRoot))
                        {
                            slot.bonesResolved = false;
                            slot.boneHits = 0;
                            slot.rigRoot = nullptr;
                            slot.headIsBase = false;
                            slot.stretchStreak = 0;
                            slot.rigCooldown = now + 500;
                            TraceLine("rig: dropped, torso stretched onto another body");
                            continue;
                        }
                        for (int bone = 0; bone < kBoneCount; ++bone)
                        {
                            slot.bonePos[bone] = previousPosition[bone];
                            slot.boneOk[bone] = previousOk[bone];
                        }
                        char held[96];
                        std::snprintf(held, sizeof(held),
                                      "rig: torso stretched, pose held (streak %u)",
                                      static_cast<unsigned>(slot.stretchStreak));
                        TraceLine(held);
                        continue;
                    }
                    slot.stretchStreak = 0;

                    // Посегментная проверка цепей рук (на каждом BonePass).
                    const int armChains[2][3] = {
                        { BoneLeftUpperArm,  BoneLeftLowerArm,  BoneLeftHand  },
                        { BoneRightUpperArm, BoneRightLowerArm, BoneRightHand }
                    };
                    for (int s = 0; s < 2; ++s)
                    {
                        if (haveTorso && slot.boneOk[armChains[s][0]] &&
                            aim::Length(aim::Sub(slot.bonePos[armChains[s][0]], torso)) > 0.75f)
                        {
                            slot.boneOk[armChains[s][0]] = false;
                            slot.boneOk[armChains[s][1]] = false;
                            slot.boneOk[armChains[s][2]] = false;
                            continue;
                        }
                        if (slot.boneOk[armChains[s][0]] && slot.boneOk[armChains[s][1]] &&
                            aim::Length(aim::Sub(slot.bonePos[armChains[s][1]], slot.bonePos[armChains[s][0]])) > 0.55f)
                        {
                            slot.boneOk[armChains[s][1]] = false;
                            slot.boneOk[armChains[s][2]] = false;
                            continue;
                        }
                        if (slot.boneOk[armChains[s][1]] && slot.boneOk[armChains[s][2]] &&
                            aim::Length(aim::Sub(slot.bonePos[armChains[s][2]], slot.bonePos[armChains[s][1]])) > 0.55f)
                        {
                            slot.boneOk[armChains[s][2]] = false;
                        }
                    }
                }

                // Anchor captured at the top of this pass, together with the
                // joints: poseFeet and bonePos[] describe one instant.
                slot.poseTick = NowMicroseconds();
                if (haveCaptureFeet)
                {
                    slot.poseFeet = captureFeet;
                    slot.lastFeet = captureFeet;
                    slot.haveFeet = true;
                }
                if (read > 0 && failed == read)
                {
                    // Every joint of this body failed to read: the rig is gone
                    // (respawn, pooled character). One flaky joint used to hit
                    // the old one-third rule and throw away the WHOLE skeleton
                    // for a minute — that is the figure vanishing on characters
                    // seated in cars. Partial misses keep the rig; the failed
                    // joints hold their last screen position instead.
                    slot.bonesResolved = false;
                    slot.boneHits = 0;
                    slot.rigRoot = nullptr;
                    slot.headIsBase = false;
                    // A pose switch makes all 19 reads fail for one pass while
                    // the component is still alive. Retry in 500 ms then, and
                    // keep the long back-off for a component that is really gone.
                    slot.rigCooldown = now + (Alive(slot.transform) ? 500ull : kRigRetryMs);
                }
            }
        }

        // --- nameplate probe -------------------------------------------------
        // Nickname, role and armor live on BattleCharacterModel, and no
        // MonoBehaviour in DummyDll/dump.cs holds one — the server model is not
        // reachable from the component list the esp already walks. The nameplate
        // view is the only client-side object that carries a nickname, so its
        // layout, its owner chain and its screen anchor are recorded once, to
        // evidence, instead of being guessed at. One file, one write, no part of
        // the draw path.
        bool g_namesProbed = false;

        bool ReadRaw(uintptr_t address, void *out, size_t size)
        {
            if (address == 0 || out == nullptr || size == 0)
            {
                return false;
            }
            return ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<LPCVOID>(address), out,
                                     size, nullptr) != FALSE;
        }

        void *ReadPointer(void *base, size_t offset)
        {
            void *value = nullptr;
            if (base == nullptr || offset == static_cast<size_t>(-1) ||
                !ReadRaw(reinterpret_cast<uintptr_t>(base) + offset, &value, sizeof(value)))
            {
                return nullptr;
            }
            return value;
        }

        void AppendPointer(std::string &body, const char *prefix, const void *value)
        {
            char line[128];
            std::snprintf(line, sizeof(line), "%s0x%llX\n", prefix,
                          static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(value)));
            body += line;
        }

        void ProbeNameplates()
        {
            if (g_namesProbed)
            {
                return;
            }
            // FindClassAnyImage walks every metadata image; on a screen where the
            // class is not loaded it costs ~16 ms and the old code paid it on
            // EVERY scan (3 s) — that is the blink Olivia sees. A miss is
            // remembered for half a minute.
            static void *cachedClass = nullptr;
            static ULONGLONG retryAt = 0;
            if (cachedClass == nullptr)
            {
                if (GetTickCount64() < retryAt)
                {
                    return;
                }
                cachedClass = g_resolver->FindClassAnyImage(
                    "Game.Battle.EntityModel.BattleCharacter.Stats",
                    "CharacterOverlayStatView");
                if (cachedClass == nullptr)
                {
                    retryAt = GetTickCount64() + 30000;
                    return;   // loaded with the battle scene; try again next scan
                }
            }
            void *klass = cachedClass;

            std::string body = "-- nameplates --\n";
            char line[256];

            // Screen size for the projection below. Screen.get_width() is a
            // static property, so no instance is needed.
            float displayWidth = 1920.0f;
            float displayHeight = 1080.0f;
            void *screenClass = g_resolver->FindClassAnyImage("UnityEngine", "Screen");
            if (screenClass != nullptr)
            {
                void *getWidth = g_resolver->Method(screenClass, "get_width", 0);
                void *getHeight = g_resolver->Method(screenClass, "get_height", 0);
                void *boxed = getWidth != nullptr ? Invoke1(getWidth, nullptr, nullptr) : nullptr;
                if (boxed != nullptr)
                {
                    const void *value = g_resolver->Unbox(boxed);
                    if (value != nullptr)
                    {
                        displayWidth = static_cast<float>(*static_cast<const int32_t *>(value));
                    }
                }
                boxed = getHeight != nullptr ? Invoke1(getHeight, nullptr, nullptr) : nullptr;
                if (boxed != nullptr)
                {
                    const void *value = g_resolver->Unbox(boxed);
                    if (value != nullptr)
                    {
                        displayHeight = static_cast<float>(*static_cast<const int32_t *>(value));
                    }
                }
            }
            std::snprintf(line, sizeof(line), "screen %.0fx%.0f\n", displayWidth, displayHeight);
            body += line;

            const size_t offLabel = g_resolver->FieldOffset(klass, "<Nickname>k__BackingField");
            const size_t offObject = g_resolver->FieldOffset(klass, "_nickname");
            const size_t offRect = g_resolver->FieldOffset(klass, "_rectTransform");
            std::snprintf(line, sizeof(line), "offset label=%zu object=%zu rect=%zu\n", offLabel,
                          offObject, offRect);
            body += line;

            void *type = g_resolver->TypeObjectOf(klass);
            void *array = type != nullptr ? Invoke1(m_findObjects, nullptr, type) : nullptr;
            if (array == nullptr)
            {
                body += "find failed\n";
                WriteDebugFile(L"OmegaCore_names.txt", body);
                return;
            }
            const size_t found = std::min(g_resolver->ArrayLength(array), kMaxEntities);
            if (found == 0)
            {
                // Still in the menu: the plates live on the battle scene, so the
                // probe waits for the world instead of burning its one shot on
                // an empty list.
                return;
            }
            g_namesProbed = true;
            std::snprintf(line, sizeof(line), "found=%zu\n", found);
            body += line;

            void *tmpClass = g_resolver->FindClassAnyImage("TMPro", "TextMeshProUGUI");
            void *tmpType = tmpClass != nullptr ? g_resolver->TypeObjectOf(tmpClass) : nullptr;
            void *getText = tmpClass != nullptr ? g_resolver->Method(tmpClass, "get_text", 0) : nullptr;
            void *goClass = g_resolver->FindClassAnyImage("UnityEngine", "GameObject");
            void *getComponent = goClass != nullptr ? g_resolver->Method(goClass, "GetComponent", 1)
                                                    : nullptr;
            std::snprintf(line, sizeof(line), "tmp=%s getComponent=%s\n",
                          getText != nullptr ? "yes" : "no", getComponent != nullptr ? "yes" : "no");
            body += line;

            if (!RefreshCamera() || !RefreshMatrices())
            {
                body += "camera: not ready\n";
            }

            for (size_t i = 0; i < found && i < 12; ++i)
            {
                void *component = g_resolver->ArrayAt(array, i);
                if (component == nullptr)
                {
                    continue;
                }
                std::snprintf(line, sizeof(line), "\n-- view %zu --\n", i);
                body += line;
                AppendPointer(body, "component ", component);

                void *transform = Invoke1(m_getTransform, component, nullptr);
                AppendPointer(body, "transform ", transform);
                Vec3 world{};
                if (transform != nullptr && TransformPosition(transform, world))
                {
                    float sx = 0.0f;
                    float sy = 0.0f;
                    float depth = 0.0f;
                    const bool projected =
                        Project(world, displayWidth, displayHeight, sx, sy, depth);
                    std::snprintf(line, sizeof(line),
                                  "world %.2f %.2f %.2f  screen %.1f %.1f  depth %.2f%s\n",
                                  world.x, world.y, world.z, sx, sy, depth,
                                  projected ? "" : "  (outside)");
                    body += line;
                }
                // The chain above the plate is what says whose plate it is.
                void *node = transform;
                for (int level = 0; level < 8 && node != nullptr; ++level)
                {
                    void *name = Invoke1(m_getName, node, nullptr);
                    const std::string text =
                        name != nullptr ? g_resolver->ReadString(name, 40) : std::string();
                    std::snprintf(line, sizeof(line), "  ^%d %s\n", level,
                                  text.empty() ? "(?)" : text.c_str());
                    body += line;
                    node = m_getParent != nullptr ? Invoke1(m_getParent, node, nullptr) : nullptr;
                }

                // TextLabel<string,string>: a nickname is an argument, not a
                // localization entry, so the argument list is where it lives.
                // The offsets are not in the dump for a generic instance, so
                // every plausible field is tried and only readable strings land
                // in the file.
                void *label = ReadPointer(component, offLabel);
                AppendPointer(body, "label ", label);
                for (size_t field = 0x10; field <= 0x28; field += 8)
                {
                    void *value = ReadPointer(label, field);
                    if (value == nullptr)
                    {
                        continue;
                    }
                    std::snprintf(line, sizeof(line), "  field 0x%zX -> ", field);
                    body += line;
                    AppendPointer(body, "", value);
                    const std::string text = g_resolver->ReadString(value, 40);
                    if (!text.empty())
                    {
                        std::snprintf(line, sizeof(line), "    string: %s\n", text.c_str());
                        body += line;
                    }
                    int32_t count = 0;
                    if (ReadRaw(reinterpret_cast<uintptr_t>(value) + 0x18, &count, sizeof(count)) &&
                        count > 0 && count <= 64)
                    {
                        void *items = ReadPointer(value, 0x10);
                        for (int32_t index = 0; items != nullptr && index < 3; ++index)
                        {
                            void *item = g_resolver->ArrayAt(items, static_cast<size_t>(index));
                            const std::string itemText =
                                item != nullptr ? g_resolver->ReadString(item, 40) : std::string();
                            std::snprintf(line, sizeof(line), "    list[%d] (count %d): %s\n", index,
                                          count, itemText.empty() ? "(?)" : itemText.c_str());
                            body += line;
                        }
                    }
                }

                // The plate object itself: its name, and the text the game draws
                // in it.
                void *plate = ReadPointer(component, offObject);
                AppendPointer(body, "plate ", plate);
                if (plate != nullptr)
                {
                    void *name = Invoke1(m_getName, plate, nullptr);
                    const std::string text =
                        name != nullptr ? g_resolver->ReadString(name, 40) : std::string();
                    std::snprintf(line, sizeof(line), "  plate name: %s\n", text.c_str());
                    body += line;
                    if (getComponent != nullptr && tmpType != nullptr && getText != nullptr)
                    {
                        void *textComponent = Invoke1(getComponent, plate, tmpType);
                        AppendPointer(body, "  tmp ", textComponent);
                        void *managed =
                            textComponent != nullptr ? Invoke1(getText, textComponent, nullptr)
                                                     : nullptr;
                        const std::string drawn =
                            managed != nullptr ? g_resolver->ReadString(managed, 40)
                                               : std::string();
                        if (!drawn.empty())
                        {
                            std::snprintf(line, sizeof(line), "  text: %s\n", drawn.c_str());
                            body += line;
                        }
                    }
                }

                // Where the plate sits on screen right now: that is what a
                // nameplate can be matched to a character by.
                void *rect = ReadPointer(component, offRect);
                if (rect != nullptr)
                {
                    void *rectClass = g_resolver->ClassOf(rect);
                    void *getAnchored =
                        rectClass != nullptr ? g_resolver->Method(rectClass, "get_anchoredPosition", 0)
                                             : nullptr;
                    void *boxed = getAnchored != nullptr ? Invoke1(getAnchored, rect, nullptr) : nullptr;
                    if (boxed != nullptr)
                    {
                        const void *value = g_resolver->Unbox(boxed);
                        if (value != nullptr)
                        {
                            float anchored[2] = {0.0f, 0.0f};
                            std::memcpy(anchored, value, sizeof(anchored));
                            std::snprintf(line, sizeof(line), "  anchored %.1f %.1f\n", anchored[0],
                                          anchored[1]);
                            body += line;
                        }
                    }
                }
            }

            body += "\n-- esp slots --\n";
            for (const Slot &slot : g_slots)
            {
                AppendPointer(body, "component ", slot.component);
                std::snprintf(line, sizeof(line), "  name %s\n", slot.name);
                body += line;
                Vec3 world{};
                if (slot.transform != nullptr && TransformPosition(slot.transform, world))
                {
                    float sx = 0.0f;
                    float sy = 0.0f;
                    float depth = 0.0f;
                    const bool projected =
                        Project(world, displayWidth, displayHeight, sx, sy, depth);
                    std::snprintf(line, sizeof(line),
                                  "  world %.2f %.2f %.2f  screen %.1f %.1f  depth %.2f%s\n", world.x,
                                  world.y, world.z, sx, sy, depth, projected ? "" : "  (outside)");
                    body += line;
                }
            }

            WriteDebugFile(L"OmegaCore_names.txt", body);
        }

        // --- nameplates -------------------------------------------------------
        // The server model that carries nickname / role / armor is not reachable
        // from any component in this build (see ROBRUN_DLC/docs/DECISIONS.md), so
        // the nickname comes from the one client-side object that already has it:
        // the world nameplate the game draws above every character. A plate is
        // bound to a body by world position, and a body without a plate simply
        // gets no nickname instead of a wrong one.
        struct Plate
        {
            Vec3 world{};
            char text[40] = "";
            // The hud canvas is screen space, so every plate reports the same
            // world position and can only be bound to a body by its projected
            // screen position. Set once per collection, from the positions
            // themselves, not from a guess about the canvas mode.
            bool screenSpace = false;
        };
        std::vector<Plate> g_plates;
        std::vector<Plate> g_publishedPlates; // worker → lock → render hand-off
        std::vector<Plate> g_renderPlates;    // render-thread's own copy

        void *ReadPointer(void *base, size_t offset);

        // List<T> layout: _items at 0x10, _size at 0x18. A string substitution
        // arrives as an Il2CppString pointer: length at 0x10, UTF-16 chars at
        // 0x14. The length is validated before the chars are touched, because a
        // wrong guess here is an access violation, and a caught fault poisons the
        // whole thread for the rest of the session.
        std::string NicknameFromArguments(void *component, size_t offLabel)
        {
            if (component == nullptr || offLabel == static_cast<size_t>(-1) || g_resolver == nullptr)
            {
                return std::string();
            }
            void *label = ReadPointer(component, offLabel);
            if (label == nullptr || !Alive(label))
            {
                return std::string();
            }
            void *list = ReadPointer(label, 0x10);
            // A List<T> is not an array: _items at 0x10, _size at 0x18. Reading
            // the list itself as an array returned a length (2) and a null entry
            // for every element, which is why every plate came back empty.
            void *items = list != nullptr ? ReadPointer(list, 0x10) : nullptr;
            if (items == nullptr)
            {
                return std::string();
            }
            const size_t length = std::min<size_t>(g_resolver->ArrayLength(items), 8);
            for (size_t i = 0; i < length; ++i)
            {
                void *entry = g_resolver->ArrayAt(items, i);
                if (entry == nullptr || !Alive(entry))
                {
                    continue;
                }
                int chars = 0;
                if (!g_resolver->ReadBlock(static_cast<char *>(entry) + 0x10, &chars, sizeof(chars)))
                {
                    continue;
                }
                if (chars <= 0 || chars > 63)
                {
                    continue;
                }
                const std::string text = g_resolver->ReadString(entry, sizeof(Plate::text) - 1);
                if (!text.empty())
                {
                    return text;
                }
            }
            return std::string();
        }

        void CollectPlates(float displayWidth, float displayHeight)
        {
            g_plates.clear();
            // Same cost as the nameplate probe: the class lookup is a scan over
            // every metadata image, so a miss is cached for 30 s instead of being
            // repeated on every scan.
            static void *cachedPlateClass = nullptr;
            static ULONGLONG plateRetryAt = 0;
            if (cachedPlateClass == nullptr)
            {
                if (GetTickCount64() < plateRetryAt)
                {
                    return;
                }
                cachedPlateClass = g_resolver->FindClassAnyImage(
                    "Game.Battle.EntityModel.BattleCharacter.Stats",
                    "CharacterOverlayStatView");
                if (cachedPlateClass == nullptr)
                {
                    plateRetryAt = GetTickCount64() + 30000;
                    return;   // loaded with the battle scene
                }
            }
            void *klass = cachedPlateClass;
            if (klass == nullptr || m_findObjects == nullptr)
            {
                Trace(klass == nullptr ? "plates: class not loaded" : "plates: no findObjects");
                return;   // loaded with the battle scene
            }
            const size_t offObject = g_resolver->FieldOffset(klass, "_nickname");
            // TextLabel<string,string> keeps the substitutions the plate is built
            // from, and the nickname is one of them. The TMP text on the plate is
            // "No translation found" for every character, so it is only a fallback.
            size_t offLabel = g_resolver->FieldOffset(klass, "<Nickname>k__BackingField");
            if (offLabel == static_cast<size_t>(-1))
            {
                offLabel = g_resolver->FieldOffset(klass, "Nickname");
            }
            void *tmpClass = g_resolver->FindClassAnyImage("TMPro", "TextMeshProUGUI");
            void *goClass = g_resolver->FindClassAnyImage("UnityEngine", "GameObject");
            void *tmpType = tmpClass != nullptr ? g_resolver->TypeObjectOf(tmpClass) : nullptr;
            void *getText = tmpClass != nullptr ? g_resolver->Method(tmpClass, "get_text", 0) : nullptr;
            void *getComponent =
                goClass != nullptr ? g_resolver->Method(goClass, "GetComponent", 1) : nullptr;
            if (offObject == static_cast<size_t>(-1) || tmpType == nullptr || getText == nullptr ||
                getComponent == nullptr)
            {
                Trace("plates: entry points missing");
                return;
            }
            void *type = g_resolver->TypeObjectOf(klass);
            if (type == nullptr)
            {
                Trace("plates: no type object");
                return;
            }
            void *array = Invoke1(m_findObjects, nullptr, type);
            if (array == nullptr)
            {
                Trace("plates: find returned null");
                return;
            }

            const size_t found = std::min(g_resolver->ArrayLength(array), kMaxEntities);
            std::string diag;
            char line[192];
            // The field-by-field probe below enumerates every field of the class,
            // builds a std::string per field and reads strings out of the live
            // objects. It is a one-shot diagnosis of a class whose offsets are
            // still unknown, and it was the bulk of the 78 ms scan that showed up
            // as a blink every few seconds. Once per process is enough.
            static bool diagDone = false;
            for (size_t i = 0; i < found; ++i)
            {
                void *component = g_resolver->ArrayAt(array, i);
                if (component == nullptr)
                {
                    continue;
                }
                void *plateObject = ReadPointer(component, offObject);
                std::string text = NicknameFromArguments(component, offLabel);
                if (!diagDone && diag.size() < 8192 && i < 4)
                {
                    // Every plate was rejected with an empty string and the only
                    // trace was "kept 0 of 11" — no way to tell a missing field
                    // from a wrong List layout. One-shot, first four plates.
                    void *label = ReadPointer(component, offLabel);
                    void *list = label != nullptr ? ReadPointer(label, 0x10) : nullptr;
                    void *items = list != nullptr ? ReadPointer(list, 0x10) : nullptr;
                    std::snprintf(line, sizeof(line),
                                  "plate %zu comp=%p offObject=%zu offLabel=%zu object=%p "
                                  "label=%p list=%p items=%p len=%zu args='%s'\n",
                                  i, component, offObject, offLabel, plateObject, label, list, items,
                                  items != nullptr ? g_resolver->ArrayLength(items) : 0u,
                                  text.c_str());
                    diag += line;
                    for (int k = 0; k < 3 && items != nullptr; ++k)
                    {
                        void *entry = g_resolver->ArrayAt(items, static_cast<size_t>(k));
                        int chars = -1;
                        std::string value;
                        if (entry != nullptr)
                        {
                            g_resolver->ReadBlock(static_cast<char *>(entry) + 0x10, &chars,
                                                  sizeof(chars));
                            value = g_resolver->ReadString(entry, 40);
                        }
                        std::snprintf(line, sizeof(line), "  arg[%d] %p chars=%d value='%s'\n", k,
                                      entry, chars, value.c_str());
                        diag += line;
                    }
                    // Offsets straight from the running runtime instead of the
                    // dump: the plate class read back empty, and a field list with
                    // live types says in one look which member holds the nickname.
                    for (const Il2CppField &field : g_resolver->Fields(klass))
                    {
                        if (field.name.empty() || field.offset == static_cast<size_t>(-1))
                        {
                            continue;
                        }
                        void *value = ReadPointer(component, field.offset);
                        char extra[96] = "";
                        // Only dereference what is known to be an object: a string
                        // and a TMP label are safe to read, anything else is just
                        // reported by its pointer.
                        if (value != nullptr &&
                            (field.typeName.find("String") != std::string::npos ||
                             field.typeName.find("TextMeshPro") != std::string::npos ||
                             field.typeName.find("TextLabel") != std::string::npos))
                        {
                            const std::string text = g_resolver->ReadString(value, 32);
                            std::snprintf(extra, sizeof(extra), " text='%s'", text.c_str());
                        }
                        std::snprintf(line, sizeof(line), "  field %s : %s @%zu -> %p%s\n",
                                      field.name.c_str(), field.typeName.c_str(), field.offset, value,
                                      extra);
                        diag += line;
                    }
                    // The nickname lives inside the TextLabel<string,string> at
                    // 0x50, and the generic class prints 0x0 offsets in the dump,
                    // so its members are listed from the live object instead.
                    void *labelObject = offLabel != static_cast<size_t>(-1)
                                            ? ReadPointer(component, offLabel)
                                            : nullptr;
                    if (labelObject != nullptr)
                    {
                        void *labelClass = g_resolver->ClassOf(labelObject);
                        std::snprintf(line, sizeof(line), "  label class=%s\n",
                                      g_resolver->ClassName(labelClass).c_str());
                        diag += line;
                        for (const Il2CppField &field : g_resolver->Fields(labelClass))
                        {
                            if (field.name.empty() || field.offset == static_cast<size_t>(-1))
                            {
                                continue;
                            }
                            void *value = ReadPointer(labelObject, field.offset);
                            char extra[96] = "";
                            if (value != nullptr &&
                                field.typeName.find("String") != std::string::npos)
                            {
                                const std::string text = g_resolver->ReadString(value, 32);
                                std::snprintf(extra, sizeof(extra), " text='%s'", text.c_str());
                            }
                            std::snprintf(line, sizeof(line), "  L field %s : %s @%zu -> %p%s\n",
                                          field.name.c_str(), field.typeName.c_str(), field.offset,
                                          value, extra);
                            diag += line;
                        }
                    }
                }
                // TMP text first: it is the value the game itself already
                // resolved and wrote into the plate. The substitution list this
                // was supposed to come from does not hold strings — every entry
                // read back as garbage — so it stays only as a fallback.
                void *tmp = nullptr;
                if (plateObject != nullptr)
                {
                    tmp = Invoke1(getComponent, plateObject, tmpType);
                    void *managed = tmp != nullptr ? Invoke1(getText, tmp, nullptr) : nullptr;
                    if (managed != nullptr)
                    {
                        const std::string candidate =
                            g_resolver->ReadString(managed, sizeof(Plate::text) - 1);
                        // Before the localization table is up every plate reads
                        // "No translation found"; that is a placeholder, not a
                        // nickname, and drawing it would be a lie on every body.
                        if (candidate.find("translation") == std::string::npos)
                        {
                            text = candidate;
                        }
                    }
                }
                if (text.empty())
                {
                    continue;
                }
                // m_getTransform is Component.get_transform(): pass a Component
                // (tmp or the plateObject itself when tmp is null), never a
                // raw GameObject. The old code passed plateObject (a GameObject)
                // and got a null transform on every call.
                void *transformSource = tmp != nullptr ? tmp : component;
                void *transform = Invoke1(m_getTransform, transformSource, nullptr);
                Plate plate;
                if (transform == nullptr || !TransformPosition(transform, plate.world))
                {
                    continue;
                }
                // Pooled inactive overlays are parked at y ≈ 10 000.
                if (!aim::Finite(plate.world.y) || std::fabs(plate.world.y) > 5000.0f)
                {
                    continue;
                }
                // Screen Space Overlay UI (UiCanvas): z == 0 and x/y inside
                // the viewport. Per-plate detection handles canvases where each
                // plate has a distinct anchor (the old all-same-pos heuristic
                // missed those and returned plates=0).
                if (std::fabs(plate.world.z) < 0.01f &&
                    displayWidth > 1.0f && displayHeight > 1.0f &&
                    plate.world.x >= -64.0f && plate.world.x <= displayWidth  + 64.0f &&
                    plate.world.y >= -64.0f && plate.world.y <= displayHeight + 64.0f)
                {
                    plate.screenSpace = true;
                }
                std::snprintf(plate.text, sizeof(plate.text), "%s", text.c_str());
                g_plates.push_back(plate);
            }

            // Legacy world-space heuristic: when every plate reports the same
            // position (old screen-space canvas variant), mark all as screenSpace.
            // The per-plate z/xy detection above already handles the common case;
            // this catches the edge case where z != 0 but all anchors cluster.
            if (g_plates.size() >= 2)
            {
                bool distinct = false;
                for (size_t i = 1; i < g_plates.size(); ++i)
                {
                    if (aim::Length(aim::Sub(g_plates[i].world, g_plates[0].world)) > 0.5f)
                    {
                        distinct = true;
                        break;
                    }
                }
                if (!distinct)
                {
                    for (Plate &plate : g_plates)
                    {
                        plate.screenSpace = true;
                    }
                }
            }
            char keptLine[64];
            std::snprintf(keptLine, sizeof(keptLine), "plates: kept %zu of %zu", g_plates.size(), found);
            Trace(keptLine);
            if (!diag.empty())
            {
                diagDone = true;
            }
            if (g_plates.empty() && !diag.empty())
            {
                static bool written = false;
                if (!written)
                {
                    written = true;
                    WriteDebugFile(L"OmegaCore_nick.txt", diag);
                }
            }
        }

        // Screen-space plates carry no world position, so the body they belong
        // to is the one drawn right under them: nearest box top, with the plate
        // above the head where the game puts it. Runs after Reproject, because
        // that is the first moment the boxes have pixel coordinates.
        // Screen-space plates carry no world position, so the body they belong
        // to is the one drawn right under them: nearest box top, with the plate
        // above the head where the game puts it. Runs at the end of Reproject,
        // which is the first moment boxes have pixel coordinates.
        // useRenderPlates: true when called from the render thread (use g_renderPlates),
        // false when called on the worker thread (use g_plates).
        // displayHeight: needed to flip Unity's bottom-left Y to DX11 top-left Y.
        void MatchScreenPlates(std::vector<Entity> &entities, bool useRenderPlates,
                               float displayHeight)
        {
            const std::vector<Plate> &plates = useRenderPlates ? g_renderPlates : g_plates;
            bool anyScreen = false;
            for (const Plate &plate : plates)
            {
                if (plate.screenSpace && plate.text[0] != '\0')
                {
                    anyScreen = true;
                    break;
                }
            }
            if (!anyScreen || displayHeight <= 1.0f)
            {
                return;
            }

            for (Entity &entity : entities)
            {
                entity.nickname[0] = '\0';
            }
            for (const Plate &plate : plates)
            {
                if (!plate.screenSpace || plate.text[0] == '\0')
                {
                    continue;
                }
                // Unity UiCanvas origin is bottom-left (Y up);
                // DX11/ImGui boxTop is top-left (Y down) — flip.
                const float plateY = displayHeight - plate.world.y;
                Entity *best = nullptr;
                float bestDistance = 220.0f;   // pixels, plate to box top
                for (Entity &entity : entities)
                {
                    if (!entity.visible || entity.boxTop <= 0.0f)
                    {
                        continue;
                    }
                    const float centreX = (entity.boxLeft + entity.boxRight) * 0.5f;
                    const float dx = plate.world.x - centreX;
                    const float dy = plateY - entity.boxTop;
                    // The plate hangs above the head: reject bodies that are
                    // below the plate (dy > 60) or way above it (dy < -160).
                    if (dy > 60.0f || dy < -160.0f)
                    {
                        continue;
                    }
                    const float distance = std::sqrt(dx * dx + dy * dy);
                    if (distance < bestDistance)
                    {
                        bestDistance = distance;
                        best = &entity;
                    }
                }
                if (best == nullptr || best->nickname[0] != '\0')
                {
                    continue;   // one name per body, first plate wins
                }
                std::snprintf(best->nickname, sizeof(best->nickname), "%s", plate.text);
            }
        }

        // A cached rig is only this character's while the mainRigBody it was
        // resolved under still hangs below the slot's own transform. The view
        // component is pooled: the same pointer comes back for a different
        // character, and the Transform comparison in Scan() cannot catch a
        // re-parented pool object that kept its transform while the body under
        // it changed. One parent walk per scanned slot is cheap next to the
        // 19 joint reads it protects.
        bool RigUnderTransform(void *transform, void *rigRoot)
        {
            if (transform == nullptr || rigRoot == nullptr || m_getParent == nullptr)
            {
                return false;
            }
            if (!Alive(rigRoot))
            {
                return false;
            }
            void *node = rigRoot;
            // 24, not 12: a body parented to a car seat sits deeper than the
            // plain character hierarchy, and a 12-step walk from the rig root
            // never reached charRoot, so ownership of a seated rig read false
            // and the rig was torn down on the next suspicious pass.
            for (int step = 0; step < 24 && node != nullptr; ++step)
            {
                if (node == transform)
                {
                    return true;
                }
                node = Invoke1(m_getParent, node, nullptr);
            }
            return false;
        }

        // The one expensive stage. Runs every scanIntervalMs and on demand from
        // the menu; a fault here keeps whatever was already cached, because the
        // previous list is still the best information available.
        void Scan()
        {
            if (g_anchor == nullptr || m_findObjects == nullptr)
            {
                return;
            }
            if (g_scanCooldownUntil != 0)
            {
                if (GetTickCount64() < g_scanCooldownUntil)
                {
                    return;
                }
                g_scanCooldownUntil = 0;
            }

            // FindObjectsOfType takes a System.Type. il2cpp hands back the same
            // Type object for the same class every time, so it is cached until the
            // anchor class changes: building it costs a managed call that
            // measured ~15 ms inside Present, which is the visible blink even on
            // a scan that found nothing.
            static void *cachedType = nullptr;
            static void *cachedTypeFor = nullptr;
            const ULONGLONG typeStarted = NowMicroseconds();
            if (cachedType == nullptr || cachedTypeFor != g_anchor)
            {
                cachedType = g_resolver->TypeObjectOf(g_anchor);
                cachedTypeFor = g_anchor;
            }
            void *type = cachedType;
            g_scanTypeCost = NowMicroseconds() - typeStarted;
            if (type == nullptr)
            {
                return;
            }

            Trace("scan: find");
            g_stage = "scan: find";
            const ULONGLONG findStarted = NowMicroseconds();
            void *array = Invoke1(m_findObjects, nullptr, type);
            g_stage = "idle";
            const ULONGLONG findCost = NowMicroseconds() - findStarted;
            g_scanFindCost = findCost;
            if (array == nullptr)
            {
                return;
            }

            const size_t found = std::min(g_resolver->ArrayLength(array), kMaxEntities);
            // The chain walk is a handful of managed calls up the camera's
            // ancestors; with nothing to filter there is no reason to pay for it
            // (lobby and menu screens have zero character views).
            if (found > 0)
            {
                BuildCameraChain();
            }
            g_selfByChain = 0;

            std::vector<Slot> next;
            next.reserve(found);
            for (size_t i = 0; i < found; ++i)
            {
                void *component = g_resolver->ArrayAt(array, i);
                if (component == nullptr || !Alive(component))
                {
                    continue;
                }

                Trace("scan: transform");
                bool raised = false;
                void *transform = Invoke1(m_getTransform, component, nullptr, &raised);
                if (raised)
                {
                    // A destroyed object mid-enumeration: the whole scan is
                    // suspect and the cached list is kept untouched.
                    g_forceScan = true;
                    return;
                }
                if (transform == nullptr)
                {
                    continue;
                }

                // Pooled characters park at y ≈ 10000 while inactive. Reading
                // their transform every scan costs Animator resolution budget
                // and spams the rig log with false far= entries. Discard before
                // any allocation so they never enter g_slots.
                {
                    Vec3 scanPos{};
                    if (TransformPosition(transform, scanPos) &&
                        (!aim::Finite(scanPos.y) || std::fabs(scanPos.y) > 5000.0f))
                    {
                        continue;
                    }
                }

                Slot slot;
                slot.component = component;
                slot.transform = transform;

                // Reuse the rig from the previous list. Resolving an Animator
                // walks the hierarchy and 19 GetBoneTransform calls per
                // character, and doing that for every slot on every scan is what
                // stalled the game for seconds. The component pointer survives
                // the walk, so an unchanged character costs nothing here.
                for (const Slot &previous : g_slots)
                {
                    // Баг 1.7: проверяем совпадение и transform — если Unity
                    // переиспользовал component из пула для другого персонажа
                    // (сменил transform), старые указатели bone[] опасны.
                    if (previous.component != component || previous.transform != transform)
                    {
                        continue;
                    }
                    slot.animator = previous.animator;
                    slot.animatorResolved = previous.animatorResolved;
                    slot.bonesResolved = previous.bonesResolved;
                    slot.boneHits = previous.boneHits;
                    slot.poseTick = previous.poseTick;
                    // The strict table's identity has to survive the rescan as
                    // well: without these two the grouping was resolved once
                    // and then thrown away every scanIntervalMs, so the figure
                    // fell back to name matching for the rest of the session.
                    slot.rigRoot = previous.rigRoot;
                    slot.headIsBase = previous.headIsBase;
                    // The node the rig walk started from, and the two failure
                    // counters, survive the rescan too: without them a body
                    // seated in a car had its ownership anchor thrown away every
                    // scan and the rig re-walked from the pooled view.
                    slot.charRoot = previous.charRoot;
                    slot.stretchStreak = previous.stretchStreak;
                    slot.rigFaults = previous.rigFaults;
                    slot.hipsDist = previous.hipsDist;
                    // The cooldown survives the rescan. Dropping it here reset the
                    // park on every scan, so a character whose rig cannot be read
                    // was retried every scan, every retry caught a fault, and the
                    // escalating backoff stretched the scan itself to ~13 s.
                    slot.rigCooldown = previous.rigCooldown;
                    slot.selfSticky = previous.selfSticky;
                    slot.selfChain = previous.selfChain;
                    // The feet anchor survives the rescan too. Without these four
                    // fields a rescan left poseFeet/haveFeet empty for a frame,
                    // and the joint offsets were re-based onto the live feet,
                    // which is exactly the "figure jumps once per scan" report.
                    slot.poseFeet = previous.poseFeet;
                    slot.lastFeet = previous.lastFeet;
                    slot.haveFeet = previous.haveFeet;
                    slot.feetTick = previous.feetTick;
                    // The object name never changes for a pooled view; keeping it
                    // also removes one managed string allocation per character
                    // per scan, and those allocations are what drags the il2cpp
                    // GC into a render-thread frame every few seconds.
                    std::snprintf(slot.name, sizeof(slot.name), "%s", previous.name);
                    for (int bone = 0; bone < kBoneCount; ++bone)
                    {
                        slot.bone[bone] = previous.bone[bone];
                        slot.bonePos[bone] = previous.bonePos[bone];
                        slot.boneOk[bone] = previous.boneOk[bone];
                    }
                    break;
                }

                // Ownership check on a rig carried over from the previous list.
                if (slot.charRoot != nullptr && !Alive(slot.charRoot))
                {
                    // The node the walk climbed to is gone (scene change, pooled
                    // object destroyed): fall back to the view itself instead of
                    // rejecting a rig that is still perfectly good.
                    slot.charRoot = nullptr;
                }
                if (slot.rigRoot != nullptr && slot.boneHits >= kHumanoidJointThreshold)
                {
                    if (!RigOwnedBySlot(slot))
                    {
                        for (int bone = 0; bone < kBoneCount; ++bone)
                        {
                            slot.bone[bone] = nullptr;
                            slot.boneHold[bone] = 0;
                            slot.boneOk[bone] = false;
                        }
                        slot.boneHits = 0;
                        slot.bonesResolved = false;
                        slot.rigRoot = nullptr;
                        slot.headIsBase = false;
                        TraceLine("rig: cached rig no longer hangs under this body");
                    }
                }

                // The transform class is only known from a live instance, and a
                // failed resolve here costs one scan, not the overlay.
                if (m_getPosition == nullptr)
                {
                    void *klass = g_resolver->ClassOf(transform);
                    if (klass != nullptr)
                    {
                        g_transformClass = klass;
                        m_getPosition = g_resolver->Method(klass, "get_position", 0);
                        m_getPositionNative =
                            g_resolver->Method(klass, "get_position_Injected", 2);
                        if (m_getParent == nullptr)
                        {
                            m_getParent = g_resolver->Method(klass, "get_parent", 0);
                        }
                    }
                }
                if (!g_positionNativeTried && g_transformClass != nullptr)
                {
                    if (m_getPositionNative == nullptr)
                    {
                        m_getPositionNative =
                            g_resolver->Method(g_transformClass, "get_position_Injected", 2);
                    }
                    g_positionNativeTried = true;
                    g_positionNative = m_getPositionNative != nullptr;
                }
                // A cooldown earned at one place must not follow the body to
                // another. A player parked boneless at a car door, then walking
                // away, used to serve out the whole timer where the failure
                // happened. One position read, and only while a park is actually
                // in force, so the normal scan pays nothing for this.
                if (slot.rigCooldown > GetTickCount64() && m_getPosition != nullptr)
                {
                    Vec3 moved{};
                    if (TransformPosition(transform, moved) && slot.haveFeet &&
                        aim::Length(aim::Sub(moved, slot.lastFeet)) > 2.0f)
                    {
                        slot.rigCooldown = 0;
                        slot.rigFaults = 0;
                        slot.stretchStreak = 0;
                        TraceLine("rig: park released, the body moved");
                    }
                }
                slot.selfChain = SelfByChain(transform);
                // Баг 2.1: selfView через float-дистанцию (не bool).
                // selfView здесь временно = false; один победитель
                // будет выбран после цикла по всем слотам.
                slot.selfView = slot.selfChain;
                // Сохраняем дистанцию FollowCamera для выбора победителя.
                // -1 = selfChain уже победил, нет смысла сравнивать.
                // Поле временно хранится в slot.lastFeet.y — нет, используем
                // локальный вектор followDists ниже.

                if (m_getName != nullptr && slot.name[0] == '\0')
                {
                    void *managed = Invoke1(m_getName, component, nullptr);
                    if (managed != nullptr)
                    {
                        const std::string text =
                            g_resolver->ReadString(managed, sizeof(slot.name) - 1);
                        if (!text.empty())
                        {
                            std::snprintf(slot.name, sizeof(slot.name), "%s", text.c_str());
                        }
                    }
                }

                next.push_back(slot);
            }

            // Баг 2.1: выбираем ЕДИНСТВЕННОГО победителя selfView по
            // FollowCamera, если selfChain не сработал ни у кого.
            // Победителю нужен отрыв >= 1.0 м от второго кандидата
            // (нет неоднозначности в толпе). Только этому одному — selfView=true.
            {
                bool anyChain = false;
                for (const Slot &s : next) { if (s.selfChain) { anyChain = true; break; } }

                if (!anyChain && g_cameraPositionValid)
                {
                    float bestDist  = 1.0e9f;
                    float secondDist = 1.0e9f;
                    int   bestIdx   = -1;

                    for (int i = 0; i < static_cast<int>(next.size()); ++i)
                    {
                        const float d = SelfFollowCameraDistance(next[i].component);
                        if (d < bestDist)
                        {
                            secondDist = bestDist;
                            bestDist   = d;
                            bestIdx    = i;
                        }
                        else if (d < secondDist)
                        {
                            secondDist = d;
                        }
                    }
                    // Принимаем только если: в радиусе 10 м И отрыв >= 1 м.
                    constexpr float kSelfFollowRadius = 10.0f;
                    if (bestIdx >= 0 && bestDist < kSelfFollowRadius &&
                        (secondDist - bestDist) >= 1.0f)
                    {
                        next[bestIdx].selfView = true;
                    }
                }
            }

            g_slots.swap(next);
            g_forceScan = false;
            g_faultStreak = 0;
            // A scan that completed is the proof the managed side is healthy
            // again: the two-fault budget starts over instead of counting
            // faults from the beginning of the session.
            g_scanFaults = 0;
            g_scanCooldownUntil = 0;
            ProbeNameplates();
            Trace("scan: done");
        }

        // Cheap stage: one Transform.get_position per cached slot, one matrix
        // refresh, native projection. No allocations, no scene-graph walk.
        // The view slot that belongs to a model character, found by the joints
        // the slot already has in hand: a chest sits ~1.2 m over the published
        // feet, and two characters are never that close. Used only until the
        // shared hitbox buffer carries the pose of every character.
        struct BridgeMatch
        {
            Slot *slot = nullptr;
            float horizontal = -1.0f;  // nearest candidate, even when rejected
            float vertical = 0.0f;     // anchor height over the model root
        };

        // The view slot that belongs to a model character. Only the joints are
        // usable for this: slot.poseFeet used to freeze in model mode (the
        // legacy walk does not run there), measured 14-85 m away on a live
        // server, so the skeleton was anchored to a dead point and drew off the
        // body. poseFeet is stamped from the model feet now, but the joint
        // positions from the budgeted bone pass stay the matching key - they
        // need no assumption about which source is current.
        //
        // Anchors are ranked hips -> chest -> head, each with the height it sits
        // over the character root. The vertical term is weighted because a
        // crowded lobby is where two bodies are closest horizontally.
        // `claimed` makes the assignment one-to-one: the model list is longer
        // than the view list (25 vs 22 live), and without it two characters
        // wear the same skeleton.
        BridgeMatch NearestSlot(const Vec3 &feet, bool *claimed)
        {
            BridgeMatch result;
            float bestScore = 0.0f;
            size_t bestIndex = 0;

            // A pooled view component can carry a neighbour's rig for one scan.
            // Matching a character to a slot by a 1.2 m radius was loose enough
            // that the wrong slot won in a crowd, and the character then wore
            // that rig's joints until the next pass. A slot whose rig actually
            // resolved (>= a humanoid threshold of joints) is trusted at 0.85 m;
            // an unresolved slot keeps the old radius so a fresh character is
            // still bridged before its rig exists.
            constexpr float kMaxHorizontal = 0.85f;
            constexpr float kMaxHorizontalLoose = 0.75f;
            constexpr float kVerticalSlack = 0.9f;
            constexpr float kVerticalWeight = 2.0f;

            const size_t slots = std::min(g_slots.size(), kMaxEntities);
            for (size_t index = 0; index < slots; ++index)
            {
                Slot &slot = g_slots[index];
                if (claimed != nullptr && claimed[index])
                {
                    continue;
                }
                Vec3 anchor{};
                float expected = 0.0f;
                if (slot.boneOk[BoneHips])
                {
                    anchor = slot.bonePos[BoneHips];
                    expected = 0.95f;
                }
                else if (slot.boneOk[BoneChest])
                {
                    anchor = slot.bonePos[BoneChest];
                    expected = 1.25f;
                }
                else if (slot.boneOk[BoneHead])
                {
                    anchor = slot.bonePos[BoneHead];
                    expected = 1.60f;
                }
                else
                {
                    continue;
                }
                const float dx = anchor.x - feet.x;
                const float dz = anchor.z - feet.z;
                const float horizontal = std::sqrt(dx * dx + dz * dz);
                const float vertical = anchor.y - feet.y;
                if (result.horizontal < 0.0f || horizontal < result.horizontal)
                {
                    result.horizontal = horizontal;
                    result.vertical = vertical;
                }
                const float gate = slot.boneHits >= kHumanoidJointThreshold ? kMaxHorizontal
                                                                            : kMaxHorizontalLoose;
                if (horizontal > gate || std::fabs(vertical - expected) > kVerticalSlack)
                {
                    continue;
                }
                // A rig with more joints measured is better evidence that this
                // slot is the body; the bonus is small (<= 0.38) so it only
                // breaks near-ties and never bridges a body that is not there.
                const float confidence =
                    0.02f * static_cast<float>(std::min(slot.boneHits, kBoneCount));
                const float score =
                    horizontal + std::fabs(vertical - expected) * kVerticalWeight - confidence;
                if (result.slot == nullptr || score < bestScore)
                {
                    bestScore = score;
                    result.slot = &slot;
                    bestIndex = index;
                }
            }
            if (claimed != nullptr && result.slot != nullptr)
            {
                claimed[bestIndex] = true;
            }
            return result;
        }

        // Player_Woman(Clone) ships without M_head_2_jnt: the head slot points
        // at M_head_1_jnt, which is the skull base, and every head-derived
        // drawing (circle, box top, nickname, distance) hangs off it. Lifting
        // the point 0.21 m along the hips->neck axis puts it on the crown. The
        // 0.21 m is the male M_head_1_jnt -> M_head_2_jnt distance measured in
        // skelet/муж.txt and matches the live female rig within a centimetre.
        void CrownFromBase(Vec3 *bones, bool *visible, bool headIsBase)
        {
            if (!headIsBase || !visible[BoneHead] || !visible[BoneNeck] || !visible[BoneHips])
            {
                return;
            }
            const Vec3 &hipsPoint = bones[BoneHips];
            const Vec3 &neckPoint = bones[BoneNeck];
            const float ux = neckPoint.x - hipsPoint.x;
            const float uy = neckPoint.y - hipsPoint.y;
            const float uz = neckPoint.z - hipsPoint.z;
            const float span = std::sqrt(ux * ux + uy * uy + uz * uz);
            if (span <= 0.05f)
            {
                return;
            }
            const float lift = 0.21f;
            bones[BoneHead] = Vec3{ neckPoint.x + ux / span * lift,
                                    neckPoint.y + uy / span * lift,
                                    neckPoint.z + uz / span * lift };
        }

        // The rig's own head node is not trustworthy on every body: on some it
        // sits a head-height above the skull (the circle floats off the crown)
        // and on others it never resolves at all (a bare line runs up into the
        // sky with no head on it). Anatomy fixes both: crown = neck + 0.34 of
        // the hip-to-neck span. The measured node is kept only while it lands
        // inside the anatomical 10..35 cm band; outside it the overshoot is
        // clamped, not swapped, so the crown (and the box top and nickname that
        // follow it) does not jump when a head nods or aims.
        void AnatomicalCrown(Vec3 *bones, bool *visible)
        {
            if (!visible[BoneNeck] || !visible[BoneHips])
            {
                return;
            }
            const Vec3 &hipsPoint = bones[BoneHips];
            const Vec3 &neckPoint = bones[BoneNeck];
            const float ux = neckPoint.x - hipsPoint.x;
            const float uy = neckPoint.y - hipsPoint.y;
            const float uz = neckPoint.z - hipsPoint.z;
            const float span = std::sqrt(ux * ux + uy * uy + uz * uz);
            if (span <= 0.05f)
            {
                return;
            }
            float along = 0.0f;
            if (visible[BoneHead])
            {
                const Vec3 &headPoint = bones[BoneHead];
                along = ((headPoint.x - neckPoint.x) * ux +
                         (headPoint.y - neckPoint.y) * uy +
                         (headPoint.z - neckPoint.z) * uz) / span;
            }
            // The neck->crown offset is a head length, in metres, not a
            // fraction of the torso: a seated character whose hips joint
            // resolves near the seat gives a short span (crown lands inside the
            // skull), and one whose hips resolve to the feet gives a long one
            // (the circle was drawn over the car roof).
            const float crownOffset = std::clamp(span * 0.34f, 0.10f, 0.32f);
            if (!visible[BoneHead])
            {
                bones[BoneHead] = Vec3{ neckPoint.x + ux / span * crownOffset,
                                        neckPoint.y + uy / span * crownOffset,
                                        neckPoint.z + uz / span * crownOffset };
                visible[BoneHead] = true;
                return;
            }
            // Correct the OVERSHOOT, not the whole point: the measured crown is
            // moved back along the spine by exactly the distance it left the
            // band, and its perpendicular part is kept. Swapping the position
            // for the anatomical estimate the moment `along` crossed the
            // threshold made the crown jump every few frames on a nodding head.
            //
            // The residual also has to be bounded: a head node that is 40 cm to
            // the side of the neck axis is a broken joint, not a head, and
            // keeping that part intact drew the spine as a right angle into the
            // shoulder (live screenshot: the woman in the middle, crown a
            // head-width left of the neck). 12 cm is the widest a real skull
            // sits off the torso axis.
            const float limit = std::clamp(along, 0.10f, 0.35f);
            const float shift = (limit - along) / span;
            const Vec3 headPoint = bones[BoneHead];
            Vec3 corrected{ headPoint.x + ux * shift,
                            headPoint.y + uy * shift,
                            headPoint.z + uz * shift };
            const Vec3 axial{ neckPoint.x + ux / span * limit,
                              neckPoint.y + uy / span * limit,
                              neckPoint.z + uz / span * limit };
            const float lx = corrected.x - axial.x;
            const float ly = corrected.y - axial.y;
            const float lz = corrected.z - axial.z;
            const float lateral = std::sqrt(lx * lx + ly * ly + lz * lz);
            constexpr float kMaxHeadLateral = 0.12f;
            if (lateral > kMaxHeadLateral)
            {
                const float scale = kMaxHeadLateral / lateral;
                corrected = Vec3{ axial.x + lx * scale,
                                  axial.y + ly * scale,
                                  axial.z + lz * scale };
            }
            bones[BoneHead] = corrected;
        }

        void Sample(float displayWidth, float displayHeight)

        {
            g_debug[0] = '\0';

            Trace("sample: camera");
            g_stage = "sample: camera";
            const bool cameraReady = RefreshCamera() && RefreshMatrices();
            g_stage = "sample: probe";
            if (!cameraReady)
            {
                // The entities are NOT dropped here: a camera that is missing
                // for one sample (scene streaming, a menu turn) used to blank
                // the whole overlay for a frame, which reads as "the esp blinks".
                // The projection keeps running from the last world positions.
                std::snprintf(g_debug, sizeof(g_debug), "sample: no main camera");
                return;
            }

            struct Point
            {
                void *component = nullptr;
                void *animator = nullptr;
                Slot *slot = nullptr;
                Vec3 feet{};
                Vec3 anchorOffset{};
                bool selfView = false;
                bool selfChain = false;
                const char *role = "";
                const char *nickname = "";
                int armor = -1;
                int health = -1;
            };
            static Point sPoints[kMaxEntities];

            // The battle model graph, when it has been located, is the only list
            // of real characters: one entry per character, the local player
            // already excluded by the game's own pointer, no FindObjectsOfType.
            // The view slots stay alive only to hand the skeleton its joints
            // until the shared hitbox buffer carries them.
            // Ready() alone: an empty character list is a valid answer (the graph
            // is live, the player is alone in the world), not a reason to fall
            // back to the managed walk. Gating on Count() > 0 turned "only you in
            // the world" into FindObjectsOfType every 5 s — pid 3308: 95 scans,
            // 24-48 ms each, none of them finding anything the graph lacked.
            const bool modelMode = model::Ready() || model::Requested();

            const size_t limit = std::min(g_slots.size(), kMaxEntities);
            size_t count = 0;
            size_t failed = 0;

            if (modelMode)
            {
                const size_t found = std::min(model::Count(), kMaxEntities);
                // One view slot can only hold one body: claim as we go, so a
                // longer model list (25 vs 22 live) never dresses two characters
                // in the same skeleton.
                static bool claimed[kMaxEntities];
                std::memset(claimed, 0, sizeof(claimed));
                for (size_t i = 0; i < found; ++i)
                {
                    const model::Character *character = model::At(i);
                    if (character == nullptr)
                    {
                        continue;
                    }
                    sPoints[count].component = character->character;
                    sPoints[count].animator = nullptr;
                    const BridgeMatch bridge = NearestSlot(character->feet, claimed);
                    sPoints[count].slot = bridge.slot;
                    // The skeleton lives or dies on this bridge: say out loud how
                    // far the nearest view slot is, so "bones gone" is a number in
                    // the trace instead of a guess.
                    static ULONGLONG lastBridgeLog = 0;
                    if (count < 4 && GetTickCount64() - lastBridgeLog >= 1000)
                    {
                        lastBridgeLog = GetTickCount64();
                        char text[160];
                        std::snprintf(text, sizeof(text), "bridge: #%zu dxz=%.2f dy=%.2f slot=%d",
                                      count, static_cast<double>(bridge.horizontal),
                                      static_cast<double>(bridge.vertical),
                                      bridge.slot != nullptr ? 1 : 0);
                        TraceLine(text);
                    }
                    // The joints are offsets from the rig root (BonePass stamps
                    // poseFeet from exactly that node), while `feet` stays the
                    // model root: it is the ground point the box, the header and
                    // the distance are built from. Draw the joints from
                    // `feet + anchorOffset` so they land on their own anchor, and
                    // the box keeps standing on the ground. Using the rig root as
                    // `feet` would lift the box bottom to the hips; using the
                    // model root as the joint base shifts the whole figure by
                    // (rig root - model root), whose variation over a stride is
                    // the skeleton leading the runner.
                    Vec3 baseFeet = character->feet;
                    sPoints[count].anchorOffset = Vec3{};
                    if (bridge.slot != nullptr && bridge.slot->rigRoot != nullptr &&
                        TransformPosition(bridge.slot->rigRoot, baseFeet))
                    {
                        const Vec3 delta = aim::Sub(baseFeet, character->feet);
                        // A pooled rig root that now belongs to another body
                        // would drag this figure across the map: only trust a
                        // root that stands at this character.
                        if (aim::Length(delta) <= 4.0f)
                        {
                            sPoints[count].anchorOffset = delta;
                            g_anchorDelta = std::max(g_anchorDelta, aim::Length(delta));
                        }
                    }
                    sPoints[count].feet = character->feet;
                    // BonePass stamps poseFeet from lastFeet so the joints can be
                    // re-based against the feet that were current when they were
                    // read. In model mode the legacy walk never runs, so lastFeet
                    // was frozen at whatever the last view-slot read left there -
                    // poseFeet pointed at a position from an unrelated moment and
                    // every joint picked up a 10-30 cm offset. Sample now runs
                    // before BonePass, so this is the feet of the same tick.
                    if (bridge.slot != nullptr)
                    {
                        bridge.slot->lastFeet = baseFeet;
                        bridge.slot->haveFeet = true;
                        bridge.slot->feetTick = GetTickCount64();
                    }
                    sPoints[count].selfView = false;
                    sPoints[count].role = character->role;
                    sPoints[count].nickname = character->nickname;
                    sPoints[count].armor = character->armor;
                    sPoints[count].health = character->health;
                    ++count;
                }
            }
            else
            {
                for (size_t i = 0; i < limit; ++i)
                {
                    Vec3 feet{};
                    Trace("sample: position");
                    g_stage = "sample: position";
                    if (!TransformPosition(g_slots[i].transform, feet))
                    {
                        // One missed read used to delete that player's box and
                        // skeleton for a sample, which reads as "the esp drops on
                        // some players". The last good position is still the best
                        // answer for a couple of hundred milliseconds.
                        const ULONGLONG age = GetTickCount64() - g_slots[i].feetTick;
                        if (g_slots[i].haveFeet && g_slots[i].feetTick != 0 && age <= 250)
                        {
                            feet = g_slots[i].lastFeet;
                        }
                        else
                        {
                            ++failed;
                            continue;
                        }
                    }
                    else
                    {
                        g_slots[i].lastFeet = feet;
                        g_slots[i].haveFeet = true;
                        g_slots[i].feetTick = GetTickCount64();
                    }
                    g_stage = "sample: probe";
                    sPoints[count].component = g_slots[i].component;
                    sPoints[count].animator = g_slots[i].animator;
                    sPoints[count].slot = &g_slots[i];
                    sPoints[count].feet = feet;
                    // Same rule as the model path: the joints are offsets from
                    // the rig root, the box is built from this view position.
                    sPoints[count].anchorOffset = Vec3{};
                    if (g_slots[i].rigRoot != nullptr)
                    {
                        Vec3 rig{};
                        if (TransformPosition(g_slots[i].rigRoot, rig))
                        {
                            const Vec3 delta = aim::Sub(rig, feet);
                            if (aim::Length(delta) <= 4.0f)
                            {
                                sPoints[count].anchorOffset = delta;
                            }
                        }
                    }
                    sPoints[count].selfView = g_slots[i].selfView;
                    sPoints[count].selfChain = g_slots[i].selfChain;
                    sPoints[count].role = "";
                    sPoints[count].nickname = "";
                    sPoints[count].armor = -1;
                    ++count;
                }
                // Every slot unreadable at once is a scene change: the lobby was
                // left, the whole character list belongs to the old scene. Throw
                // it away once and ask for a rescan instead of walking twenty dead
                // wrappers on every sample until the timer comes round.
                if (count == 0 && failed > 0 && !g_slots.empty())
                {
                    g_slots.clear();
                    g_selfComponent = nullptr;
                    g_selfAnimator = nullptr;
                    // New scene, new bodies: the pooled address of the old own
                    // character can belong to anything now, so the latch is
                    // released with the list that produced it.
                    g_selfLocked = false;
                    g_forceScan = true;
                    g_entities.clear();
                    TraceLine("scene: slot list dropped (all unreadable)");
                    return;
                }
            }

            // The local player is every entity whose character root owns the
            // main camera: the third-person rig is parented under it. Matching
            // the Animator pointer is exact, and unlike the old "drop the
            // nearest" rule it removes ALL views of the local character — a
            // character usually carries more than one scanned view, so dropping
            // a single slot left a second box on the own body.
            void *ownedAnimator = g_options.selfFilter ? CameraAnimator() : nullptr;
            bool selfDropped = false;
            const ULONGLONG now = GetTickCount64();

            // Latch hygiene: a scene change frees the own body, and a freed
            // address can be handed out again. A dead latch is no latch.
            if (g_selfComponent != nullptr && !Alive(g_selfComponent))
            {
                g_selfComponent = nullptr;
            }
            if (g_selfAnimator != nullptr && !Alive(g_selfAnimator))
            {
                g_selfAnimator = nullptr;
            }
            // Both pointers of a latch gone: the character this latch described
            // no longer exists, so the next identification is free to pick again.
            if (g_selfLocked && g_selfComponent == nullptr && g_selfAnimator == nullptr)
            {
                g_selfLocked = false;
                TraceLine("self: latch released (own body gone)");
            }

            // Nearest body to the eye: diagnostics only, and the number the self
            // range is tuned against. In third person the own body is always the
            // closest one.
            float nearest = -1.0f;
            if (g_cameraPositionValid)
            {
                for (size_t i = 0; i < count; ++i)
                {
                    const float distance =
                        aim::Length(aim::Sub(sPoints[i].feet, g_cameraPosition));
                    if (aim::Finite(distance) && (nearest < 0.0f || distance < nearest))
                    {
                        nearest = distance;
                    }
                }
            }

            // Third fallback, used only when none of the exact tests identified
            // the own body: the third-person camera sits a fixed few metres
            // behind the player, so the CLOSEST body inside selfDistance is the
            // local one. Only that single body is dropped - never a radius
            // around the eye, which ate real players.
            //   It used to run ALWAYS because gating it on `g_selfByChain == 0`
            // let one lucky name match disable it for the whole session. That is
            // a real failure and it is why the gate is still here - but "closest
            // body to the camera" is ALSO the correct answer for the player you
            // just drove into, and a gate that runs beside an exact match cannot
            // tell the two apart. It now runs only when the exact pass came back
            // empty: exact beats guess, always.
            //   A distance guess must never LATCH. It used to write
            // g_selfLocked and g_selfComponent here, and once a neighbour was
            // the nearest body at the moment of the first sample, that latch was
            // permanent: the exact test is guarded by `!g_selfLocked`, so it could
            // not take the answer back, and the box stayed on that neighbour for
            // the rest of the session while the own body was drawn. The guess is
            // now a one-frame filter, nothing more.
            // A latch with no exit is a permanent wrong answer. The takeover
            // rule further down only lets a Transform-chain match replace it,
            // and this game never produces one - every latch in a live run
            // logs "camera" - so until now the only release was the death of
            // both pointers. View components are pooled: the same pointer comes
            // back with another character under it after a respawn or a vehicle
            // exit, and the latched body is then on the other side of the map.
            //   The release clears the latch only; the exact pass below
            // re-acquires in the same sample.
            if (g_selfLocked && g_cameraPositionValid)
            {
                // One character usually answers from MORE than one scanned view:
                // the slot list keeps a pooled view component that a respawn or a
                // vehicle exit left on the other side of the map, while the live
                // one stands under the camera. The release used to `break` on the
                // first matching view, so the stale far one fired it every scan -
                // "latch released (64.0 m)" immediately followed by "latched
                // (2.9 m)" in the same sample, over and over. The body is far
                // only when every view of it is far.
                float latchedDistance = -1.0f;
                int latchedViews = 0;
                char views[320];
                int viewsUsed = 0;
                views[0] = '\0';
                for (size_t i = 0; i < count; ++i)
                {
                    const bool same = sPoints[i].component == g_selfComponent ||
                                      (g_selfAnimator != nullptr &&
                                       sPoints[i].animator == g_selfAnimator);
                    if (!same)
                    {
                        continue;
                    }
                    const float distance =
                        aim::Length(aim::Sub(sPoints[i].feet, g_cameraPosition));
                    if (aim::Finite(distance) &&
                        (latchedDistance < 0.0f || distance < latchedDistance))
                    {
                        latchedDistance = distance;
                    }
                    ++latchedViews;
                    if (viewsUsed < 3)
                    {
                        const int used = std::snprintf(
                            views + viewsUsed, sizeof(views) - static_cast<size_t>(viewsUsed),
                            " [%u cmp=%p anim=%p %.1f m feet=%.1f,%.1f,%.1f]", static_cast<unsigned>(i),
                            sPoints[i].component, sPoints[i].animator, distance,
                            sPoints[i].feet.x, sPoints[i].feet.y, sPoints[i].feet.z);
                        if (used > 0 && viewsUsed + used < static_cast<int>(sizeof(views)))
                        {
                            viewsUsed += used;
                        }
                    }
                }
                if (latchedDistance > kSelfReleaseRadius)
                {
                    ++g_selfFarSamples;
                }
                else
                {
                    g_selfFarSamples = 0;
                }
                if (g_selfFarSamples >= kSelfFarSamples)
                {
                    g_selfFarSamples = 0;
                    char text[512];
                    std::snprintf(text, sizeof(text),
                                  "self: latch released (latched body %.1f m from the eye, "
                                  "component=%p animator=%p, %d view(s) of %u, eye=%.1f,%.1f,%.1f)"
                                  "%s",
                                  latchedDistance, g_selfComponent, g_selfAnimator,
                                  latchedViews, static_cast<unsigned>(count),
                                  g_cameraPosition.x, g_cameraPosition.y, g_cameraPosition.z,
                                  views);
                    TraceLine(text);
                    g_selfLocked = false;
                    g_selfComponent = nullptr;
                    g_selfAnimator = nullptr;
                    for (size_t k = 0; k < count; ++k)
                    {
                        if (sPoints[k].slot != nullptr)
                        {
                            sPoints[k].slot->selfSticky = false;
                        }
                    }
                }
            }

            int exactSelf = -1;
            if (g_options.selfFilter)
            {
                // Closest candidate wins. SelfByFollowCamera accepts any body
                // within 25 m of the camera (the spring arm inside a car is that
                // long), so more than one slot can answer "exact" in a crowd or a
                // car; taking the first in slot order let that pick drift.
                //
                // A live latch outranks every radius answer. The FollowCamera node
                // of a neighbour is nearer to the eye than the own body whenever
                // that neighbour stands between the camera and the player - which
                // is exactly the moment you walk up to somebody and the ESP jumps
                // onto your own character: exactSelf pointed at the neighbour,
                // the neighbour was dropped from the list, and the own body (the
                // real answer) was only kept by the component pointers. The latch
                // was built from stronger evidence and stays authoritative until
                // the release above or a Transform-chain match removes it.
                int latchedSelf = -1;
                float latchedSelfDistance = -1.0f;
                if (g_selfLocked)
                {
                    for (size_t i = 0; i < count; ++i)
                    {
                        if (!(sPoints[i].component == g_selfComponent ||
                              (g_selfAnimator != nullptr &&
                               sPoints[i].animator == g_selfAnimator)))
                        {
                            continue;
                        }
                        const float distance =
                            g_cameraPositionValid
                                ? aim::Length(aim::Sub(sPoints[i].feet, g_cameraPosition))
                                : 0.0f;
                        if (latchedSelf < 0 || (g_cameraPositionValid && distance < latchedSelfDistance))
                        {
                            latchedSelf = static_cast<int>(i);
                            latchedSelfDistance = distance;
                        }
                    }
                }
                float exactDistance = -1.0f;
                for (size_t i = 0; i < count; ++i)
                {
                    if (latchedSelf >= 0 && static_cast<int>(i) != latchedSelf)
                    {
                        continue;
                    }
                    const Slot *slot = sPoints[i].slot;
                    if (!(sPoints[i].selfView ||
                          (slot != nullptr && slot->selfSticky) ||
                          (g_selfComponent != nullptr && sPoints[i].component == g_selfComponent) ||
                          (g_selfAnimator != nullptr && sPoints[i].animator == g_selfAnimator) ||
                          (ownedAnimator != nullptr && sPoints[i].animator == ownedAnimator)))
                    {
                        continue;
                    }
                    const float distance =
                        g_cameraPositionValid
                            ? aim::Length(aim::Sub(sPoints[i].feet, g_cameraPosition))
                            : 0.0f;
                    if (exactSelf < 0 || (g_cameraPositionValid && distance < exactDistance))
                    {
                        exactSelf = static_cast<int>(i);
                        exactDistance = distance;
                    }
                }
            }

            // Exact answer: latch it, and let it replace a wrong latch - but only
            // with evidence stronger than the one already stored.
            //
            // The FollowCamera probe is not that evidence. It accepts EVERY body
            // whose camera node sits within 25 m of the eye, and in third person
            // an enemy between the camera and the player is by definition nearer
            // to the eye than the player is. Picking the closest of several
            // "exact" answers therefore flipped the latch every sample: one live
            // session alternated component ...724ca20 (63.0 m) with ...71e5db0
            // (2.9 m), and each takeover cleared the other body's sticky flag, so
            // the box left the enemy and appeared on the own character. A latch
            // that is alive and answers this test stays; only a Transform-chain
            // match - the camera's own ancestry, not a radius - may take it away.
            if (exactSelf >= 0)
            {
                void *candidate = sPoints[exactSelf].component;
                // Nearest view of the latched body, not the first one in slot
                // order: a pooled stale view of the same component would answer
                // with its own chain flag and its own distance (see the release
                // block above - the list really does hold both).
                int latched = -1;
                float latchedEye = -1.0f;
                for (size_t i = 0; i < count; ++i)
                {
                    if (sPoints[i].component == g_selfComponent ||
                        (g_selfAnimator != nullptr && sPoints[i].animator == g_selfAnimator))
                    {
                        const float distance =
                            g_cameraPositionValid
                                ? aim::Length(aim::Sub(sPoints[i].feet, g_cameraPosition))
                                : 0.0f;
                        if (latched < 0 || (g_cameraPositionValid && distance < latchedEye))
                        {
                            latched = static_cast<int>(i);
                            latchedEye = distance;
                        }
                    }
                }
                const bool latchLive = g_selfLocked && latched >= 0;
                // A candidate whose feet are tens of metres from the eye is not
                // the player under the camera, whatever its FollowCamera node
                // answers: the probe compares the node against the eye, not the
                // body. Refusing it here is what stops the latch from flipping.
                const float candidateEye =
                    g_cameraPositionValid
                        ? aim::Length(aim::Sub(sPoints[exactSelf].feet, g_cameraPosition))
                        : 0.0f;
                const bool candidateNear =
                    !g_cameraPositionValid || candidateEye <= kSelfMaxLatchDistance;
                const bool takeOver =
                    candidate != nullptr && candidateNear &&
                    (!latchLive || (!sPoints[latched].selfChain && sPoints[exactSelf].selfChain));
                if (takeOver && (!g_selfLocked || g_selfComponent != candidate))
                {
                    g_selfComponent = candidate;
                    if (sPoints[exactSelf].animator != nullptr)
                    {
                        g_selfAnimator = sPoints[exactSelf].animator;
                    }
                    if (ownedAnimator != nullptr)
                    {
                        g_selfAnimator = ownedAnimator;
                    }
                    g_selfLocked = true;
                    g_selfFarSamples = 0;
                    // A replaced latch must take its sticky flag with it: the
                    // flag is copied across rescans and would keep the old body
                    // answering "exact" on every scan, flipping the box between
                    // the two.
                    for (size_t i = 0; i < count; ++i)
                    {
                        if (sPoints[i].slot != nullptr && sPoints[i].component != candidate)
                        {
                            sPoints[i].slot->selfSticky = false;
                        }
                    }
                    char text[256];
                    std::snprintf(text, sizeof(text),
                                  "self: latched component=%p animator=%p (%s, dist=%.1f m) "
                                  "eye=%.1f,%.1f,%.1f feet=%.1f,%.1f,%.1f",
                                  g_selfComponent, g_selfAnimator,
                                  sPoints[exactSelf].selfChain ? "chain" : "camera",
                                  g_cameraPositionValid
                                      ? aim::Length(aim::Sub(sPoints[exactSelf].feet,
                                                             g_cameraPosition))
                                      : -1.0f,
                                  g_cameraPosition.x, g_cameraPosition.y,
                                  g_cameraPosition.z, sPoints[exactSelf].feet.x,
                                  sPoints[exactSelf].feet.y, sPoints[exactSelf].feet.z);
                    TraceLine(text);
                }
            }

            // Баг 2.2: при первом захвате animator мог быть nullptr
            // (ResolveAnimator ещё не вызван на этом слоте). Дозаполняем
            // g_selfAnimator при каждом Sample() пока locked && nullptr.
            if (g_selfLocked && g_selfAnimator == nullptr && exactSelf >= 0)
            {
                if (ownedAnimator != nullptr)
                {
                    g_selfAnimator = ownedAnimator;
                }
                else if (sPoints[exactSelf].component == g_selfComponent &&
                         sPoints[exactSelf].animator != nullptr)
                {
                    g_selfAnimator = sPoints[exactSelf].animator;
                }
            }

            // Distance guess: one frame, only when nothing is latched and no
            // exact answer exists.
            // Баг 2.3: при наличии g_mvpValid — всегда выбираем кандидата
            // по минимальному отклонению от центра экрана по X (в 3-м лице
            // свой персонаж всегда по центру). 3D-дистанция — tie-breaker.
            // Если MVP нет — старое поведение (расстояние + margin).
            int selfGate = -1;
            if (g_options.selfFilter && !g_selfLocked && g_cameraPositionValid && exactSelf < 0)
            {
                if (g_mvpValid)
                {
                    // Экранный центр — главный критерий.
                    float bestCenterDx = 1.0e9f;
                    float bestDist     = 1.0e9f;
                    for (size_t i = 0; i < count; ++i)
                    {
                        const float d = aim::Length(aim::Sub(sPoints[i].feet, g_cameraPosition));
                        if (!aim::Finite(d) || d > g_options.selfDistance) continue;
                        float sx = 0.0f, sy = 0.0f, depth = 0.0f;
                        if (Project(sPoints[i].feet, displayWidth, displayHeight, sx, sy, depth))
                        {
                            const float dx = std::fabs(sx - displayWidth * 0.5f);
                            if (dx < bestCenterDx || (dx == bestCenterDx && d < bestDist))
                            {
                                bestCenterDx = dx;
                                bestDist     = d;
                                selfGate     = static_cast<int>(i);
                            }
                        }
                    }
                    // Принимаем только если кандидат достаточно по центру
                    // (< 12% ширины экрана) — иначе слишком неоднозначно.
                    if (bestCenterDx > displayWidth * 0.12f)
                    {
                        selfGate = -1;
                    }
                }
                else
                {
                    // Fallback без MVP: 3D-расстояние с margin.
                    float best = g_options.selfDistance;
                    float second = 1.0e9f;
                    int bestIndex = -1;
                    for (size_t i = 0; i < count; ++i)
                    {
                        const float distance =
                            aim::Length(aim::Sub(sPoints[i].feet, g_cameraPosition));
                        if (!aim::Finite(distance))
                        {
                            continue;
                        }
                        if (distance < best)
                        {
                            second = best;
                            best = distance;
                            bestIndex = static_cast<int>(i);
                        }
                        else if (distance < second)
                        {
                            second = distance;
                        }
                    }
                    if (bestIndex >= 0 && second - best >= kSelfGateMargin)
                    {
                        selfGate = bestIndex;
                    }
                }
            }

            // A blank sample (a rescan in progress, a character pool refresh, a
            // camera swap) used to empty the entity list for a frame — that is
            // the "esp blinks" report. The previous list is kept aside and put
            // back when this sample could read nothing at all.
            static std::vector<Entity> sPrevious;
            sPrevious = g_entities;
            g_entities.clear();

            // One pose assignment per frame for the whole cast, before any box
            // is built: nearest pair first, one hitbox slot per body. Resolving
            // it per character would let the first body in the list take a slot
            // that belongs to the second.
            static std::vector<Vec3> sPoseAnchors;
            sPoseAnchors.clear();
            if (g_options.skeleton && modelMode)
            {
                sPoseAnchors.reserve(count);
                for (size_t i = 0; i < count; ++i)
                {
                    sPoseAnchors.push_back(sPoints[i].feet);
                }
                pose::Assign(sPoseAnchors.empty() ? nullptr : sPoseAnchors.data(),
                             static_cast<int>(sPoseAnchors.size()));
            }

            size_t poseOk = 0;
            size_t poseFail = 0;
            for (size_t i = 0; i < count; ++i)
            {
                // Runs in model mode too. In model mode the local body is
                // normally cut at the source by the game's own pointer, but that
                // pointer is a chain of two fields and comes back null when the
                // game renames one: pid 24832 shows "local unknown" and then
                // nothing else was filtering the own body. The geometric test
                // below only needs feet and the eye point, both available in
                // both modes, so it stays armed either way.
                // exactSelf keeps only ONE index, the closest body. The own
                // character owns several view components that all share the same
                // Animator, and any of them that was not the closest one drawn
                // through the filter. The identity pointers are exact, so every
                // slot that answers one of them is the local player.
                const bool exactMatch =
                    static_cast<int>(i) == exactSelf ||
                    (g_selfComponent != nullptr && sPoints[i].component == g_selfComponent) ||
                    (g_selfAnimator != nullptr && sPoints[i].animator == g_selfAnimator) ||
                    (ownedAnimator != nullptr && sPoints[i].animator == ownedAnimator);
                const bool gateMatch = static_cast<int>(i) == selfGate;
                const bool dropAsSelf = g_options.selfFilter && (exactMatch || gateMatch);
                if (dropAsSelf)
                {
                    if (!exactMatch)
                    {
                        // The geometric guess dropped this body for one sample.
                        // It must not touch any latched state: the body it
                        // guessed is very often the player standing next to you,
                        // and a stale latch on a real body never clears - the
                        // slot list keeps the flag across rescans
                        // (slot.selfSticky = previous.selfSticky), so that player
                        // stays invisible for the rest of the session.
                        selfDropped = true;
                        continue;
                    }
                    if (sPoints[i].selfView)
                    {
                        ++g_selfByChain;
                    }
                    // The latch itself is written in exactly one place - the
                    // takeOver block above, which checks that the candidate is
                    // alive, that a live latch only yields to a Transform-chain
                    // match, and that the body is not tens of metres from the
                    // eye. Writing the pointers again here bypassed every one of
                    // those checks: whatever exactSelf pointed at became the
                    // latch, sticky flag included.
                    selfDropped = true;
                    continue;
                }

                // One box per character: several views share one Animator, and
                // two overlapping boxes on the same body read as "uneven".
                bool duplicate = false;
                for (const Entity &existing : g_entities)
                {
                    if (sPoints[i].animator != nullptr && existing.animator == sPoints[i].animator)
                    {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate)
                {
                    continue;
                }

                // The old fixed "closer than a metre" rule is gone: selfGate
                // above already covers the own body at third-person range, and a
                // real player in melee is exactly what that radius used to eat.

                const char *name = "";
                if (sPoints[i].slot != nullptr)
                {
                    name = sPoints[i].slot->name;
                }

                Entity entity;
                entity.key = sPoints[i].component;
                entity.animator = sPoints[i].animator;
                entity.feet = sPoints[i].feet;
                entity.anchorOffset = sPoints[i].anchorOffset;
                entity.head = Vec3{ sPoints[i].feet.x, sPoints[i].feet.y + g_headOffset,
                                    sPoints[i].feet.z };

                // Nickname: the game's own slot list is authoritative and costs
                // nothing extra (one dictionary read serves every character). The
                // nameplate stays as the fallback for the legacy walk, where no
                // model character exists to ask.
                if (sPoints[i].nickname != nullptr && sPoints[i].nickname[0] != '\0')
                {
                    std::snprintf(entity.nickname, sizeof(entity.nickname), "%s", sPoints[i].nickname);
                }
                else
                {
                    float bestPlate = 2.0f;
                    for (const Plate &plate : g_plates)
                    {
                        if (plate.text[0] == '\0')
                        {
                            continue;
                        }
                        const float dx = plate.world.x - entity.feet.x;
                        const float dz = plate.world.z - entity.feet.z;
                        const float dy = plate.world.y - entity.feet.y;
                        if (dy < -1.5f || dy > 4.0f)
                        {
                            continue;
                        }
                        const float flat = dx * dx + dz * dz;
                        if (flat > bestPlate * bestPlate)
                        {
                            continue;
                        }
                        bestPlate = std::sqrt(flat);
                        std::snprintf(entity.nickname, sizeof(entity.nickname), "%s", plate.text);
                    }
                }

                // Joints come from the cached slot pose, which the budgeted
                // bone pass refreshes a couple of characters per tick.
                if (sPoints[i].role != nullptr && sPoints[i].role[0] != '\0')
                {
                    std::snprintf(entity.role, sizeof(entity.role), "%s", sPoints[i].role);
                    entity.type = RoleFromName(sPoints[i].role);
                }
                if (sPoints[i].armor >= 0)
                {
                    entity.armor = sPoints[i].armor;
                }
                entity.health = sPoints[i].health;
                // Two sources, merged, never one replacing the other: the view
                // slot carries the 19 joints of the visual rig (the only place
                // limbs exist), the shared hitbox buffer carries the five torso
                // points and is safe to read every frame. The slot fills the
                // figure, the pose then overwrites the spine.
                // Same body in the previous sample. Both bone sources need it:
                // the blend in Reproject compares the newest capture with the
                // one before it, and the history has to survive a frame in
                // which nothing new was read.
                const Entity *previousEntity = nullptr;
                if (entity.key != nullptr)
                {
                    for (const Entity &old : sPrevious)
                    {
                        if (old.key == entity.key && old.boneTick != 0)
                        {
                            previousEntity = &old;
                            break;
                        }
                    }
                }
                const bool usePose = modelMode && (pose::Live() || g_options.espV2);
                bool haveSlotFigure = false;
                // Last measured figure, carried over a gap of up to 5 s. A body
                // seated in a car loses its rig for a stretch (BonePass drops it
                // on the seating jump, SyncBonesFromSlots skips a slot with no
                // hits), and without this carry the bones vanish instead of
                // riding the body through the transition. Defined here so both
                // the pose path and the legacy path reach it: the branch used to
                // live inside `usePose`, which left the whole fallback dead
                // whenever the model graph was off.
                const auto carryPreviousFigure = [&](const Entity &previous) {
                    const ULONGLONG nowUs = NowMicroseconds();
                    if (nowUs <= previous.boneTick || nowUs - previous.boneTick > 5000000)
                    {
                        return;
                    }
                    // Keep the anchor the offsets were measured against. The
                    // offsets in previous.bones are `joint - anchor(capture)`, and
                    // the draw base is the rig root of this frame, so a rigid
                    // translation of the figure is already exact. Re-stamping the
                    // anchor to this frame's model root re-based every joint on a
                    // different node and shifted the whole carried skeleton.
                    entity.poseAnchor = previous.poseAnchor;
                    entity.boneTick = previous.boneTick;
                    entity.bonePrevTick = previous.bonePrevTick;
                    entity.bonesPrevValid = previous.bonesPrevValid;
                    entity.headIsBase = previous.headIsBase;
                    for (int bone = 0; bone < kBoneCount; ++bone)
                    {
                        entity.bones[bone] = previous.bones[bone];
                        entity.bonesPrev[bone] = previous.bonesPrev[bone];
                        entity.boneVisible[bone] = previous.boneVisible[bone];
                    }
                };
                if (g_options.skeleton && sPoints[i].slot != nullptr &&
                    sPoints[i].slot->boneHits > 0)
                {
                    const Slot &slot = *sPoints[i].slot;
                    // Capture identity. BonePass is budgeted and round-robins the
                    // slots, so most frames carry the SAME joints in bonePos as
                    // the frame before. A fresh capture shifts the offsets read
                    // last time into the history; an unchanged one keeps that
                    // history and lets the phase keep growing. Treating every
                    // frame as a new capture froze the pose for the length of
                    // the round-robin and then snapped it, because the blend
                    // restarted from the same pair every frame.
                    const unsigned long long capture = slot.poseTick;
                    const bool freshCapture =
                        previousEntity == nullptr || previousEntity->boneTick != capture;
                    if (capture == 0)
                    {
                        // Never stamped (rig resolved this frame): nothing to
                        // blend against, draw the measured pose as it is.
                        entity.boneTick = 0;
                        entity.bonesPrevValid = false;
                    }
                    else if (freshCapture)
                    {
                        entity.boneTick = capture;
                        if (previousEntity != nullptr)
                        {
                            entity.bonePrevTick = previousEntity->boneTick;
                            entity.bonesPrevValid = true;
                            for (int bone = 0; bone < kBoneCount; ++bone)
                            {
                                entity.bonesPrev[bone] = previousEntity->bones[bone];
                            }
                        }
                    }
                    else
                    {
                        entity.boneTick = previousEntity->boneTick;
                        entity.bonePrevTick = previousEntity->bonePrevTick;
                        entity.bonesPrevValid = previousEntity->bonesPrevValid;
                        for (int bone = 0; bone < kBoneCount; ++bone)
                        {
                            entity.bonesPrev[bone] = previousEntity->bonesPrev[bone];
                        }
                    }
                    // Anchor of the read, not of this frame. slot.bonePos was
                    // filled by BonePass, which stamps slot.poseFeet from
                    // slot.lastFeet in the SAME pass (BonePass). Re-basing those
                    // joints onto this frame's feet mixes two timestamps: at
                    // 10-20 m/s a pass that lands one tick late offsets every
                    // joint by 10-30 cm, which on screen is a twitch every few
                    // frames. poseFeet is the feet that were current when the
                    // joints were read, so the offsets are time-consistent and
                    // Reproject only has to translate them forward rigidly.
                    entity.poseAnchor = (slot.haveFeet && slot.poseTick != 0)
                                            ? slot.poseFeet
                                            : aim::Add(sPoints[i].feet, sPoints[i].anchorOffset);
                    for (int bone = 0; bone < kBoneCount; ++bone)
                    {
                        entity.bones[bone] = aim::Sub(slot.bonePos[bone], entity.poseAnchor);
                        entity.boneVisible[bone] = slot.boneOk[bone];
                    }
                    entity.headIsBase = slot.headIsBase;
                    // This rig has no hips joint at all ("m spine 1 jnt" is the
                    // lowest spine node), so the fork is the midpoint of the two
                    // thighs — the leg lines stay attached to the body, and the
                    // spine line has something to end on.
                    if (!entity.boneVisible[BoneHips] && entity.boneVisible[BoneLeftUpperLeg] &&
                        entity.boneVisible[BoneRightUpperLeg])
                    {
                        const Vec3 &thighLeft = entity.bones[BoneLeftUpperLeg];
                        const Vec3 &thighRight = entity.bones[BoneRightUpperLeg];
                        entity.bones[BoneHips] = Vec3{ (thighLeft.x + thighRight.x) * 0.5f,
                                                       (thighLeft.y + thighRight.y) * 0.5f,
                                                       (thighLeft.z + thighRight.z) * 0.5f };
                        entity.boneVisible[BoneHips] = true;
                    }
                    haveSlotFigure = true;
                }
                if (g_options.skeleton && usePose)
                {
                    // The pose comes from the game's shared hitbox buffer, which
                    // carries every character's joints in one table. One
                    // 252-byte read per character, no managed call. It refines
                    // the spine of a figure the slot already produced, and is the
                    // whole figure only when no slot answered.
                    pose::Figure figure{};
                    MarkStage("stage: pose sample enter");
                    // v2 asks the character itself first: its own SkeletonModel
                    // is not a shared table, so nothing can assign it the
                    // neighbour's row. When that source is empty for this body
                    // the shared table still gets its turn, so turning v2 on
                    // cannot lose a skeleton the current path would draw.
                    bool got = false;
                    if (g_options.espV2)
                    {
                        const model::Character *own = model::At(i);
                        if (own != nullptr)
                        {
                            got = pose::SampleOwn(own->character, sPoints[i].feet, figure);
                        }
                    }
                    if (!got && pose::Live())
                    {
                        got = pose::SampleAt(static_cast<int>(i), figure);
                    }
                    if (got)
                    {
                        ++poseOk;
                        // The slot index is only valid on the legacy path: the
                        // model path fills sPoints from model::Snapshot(), and
                        // g_slots can be empty there (model=1, "0 found"). The
                        // old g_slots[i] wrote through a null base and killed the
                        // process with 0xC0000005 at Sample+0x44f4, offset 0x3c
                        // (Slot::hipsDist) - read the bridge pointer instead.
                        if (sPoints[i].slot != nullptr)
                        {
                            sPoints[i].slot->hipsDist =
                                aim::Length(aim::Sub(figure.hips, sPoints[i].feet));
                        }
                        // The shared buffer is re-read every sample, so this is a
                        // fresh capture of its own unless the slot already
                        // supplied one this frame. Overwriting boneTick in the
                        // merged case would drop the limb capture identity and
                        // leave the blend dividing by the wrong interval.
                        if (!haveSlotFigure)
                        {
                            const unsigned long long capture = NowMicroseconds();
                            entity.boneTick = capture;
                            if (previousEntity != nullptr)
                            {
                                entity.bonePrevTick = previousEntity->boneTick;
                                entity.bonesPrevValid = true;
                                for (int bone = 0; bone < kBoneCount; ++bone)
                                {
                                    entity.bonesPrev[bone] = previousEntity->bones[bone];
                                }
                            }
                        }
                        // The anchor stays the one the slot's own joints were
                        // read against. Re-basing them onto this frame's feet
                        // moved every measured joint by the distance the body
                        // travelled since the bone pass - 10-30 cm at speed - and
                        // was a twitch of its own on every merged frame.
                        const Vec3 anchor =
                            haveSlotFigure
                                ? entity.poseAnchor
                                : aim::Add(sPoints[i].feet, sPoints[i].anchorOffset);
                        // The pose is published in world space by the game's
                        // table, so it still needs an anchor for the offsets and
                        // for every later carry. Without one the joints stayed
                        // absolute and the 3.5 m envelope rejected all of them -
                        // the "no skeleton at all" a seated passenger shows, who
                        // has no slot figure of his own.
                        if (!haveSlotFigure)
                        {
                            entity.poseAnchor = anchor;
                        }
                        // A joint the slot measured itself is never replaced by
                        // the shared table: that table is a pool of rows, and in
                        // a car two bodies sit close enough that the assignment
                        // hands one body the other's row for a tick. A measured
                        // joint belongs to this GameObject by construction, so
                        // the shared row only fills what is missing. The whole
                        // figure still comes from it when the slot answered
                        // nothing.
                        const bool slotOwnsFigure =
                            haveSlotFigure && sPoints[i].slot != nullptr &&
                            sPoints[i].slot->boneHits >= 12;
                        const auto put = [&](int bone, const Vec3 &world) {
                            // A slot that measured twelve or more joints already
                            // owns this figure. The shared table is a pool of
                            // rows assigned by proximity, so at 1.3-1.5 m its
                            // row can be the neighbour's; filling the torso from
                            // it puts another body's spine on this skeleton.
                            if (slotOwnsFigure || entity.boneVisible[bone])
                            {
                                return;
                            }
                            entity.bones[bone] = aim::Sub(world, anchor);
                            entity.boneVisible[bone] = true;
                        };
                        // Only a pose that IS the figure clears the rest; when
                        // the slot already supplied limbs, clearing would erase
                        // exactly the joints this merge exists to keep.
                        if (!haveSlotFigure)
                        {
                            for (int bone = 0; bone < kBoneCount; ++bone)
                            {
                                entity.boneVisible[bone] = false;
                            }
                            // The shared table publishes an actual crown bone;
                            // the skull-base lift belongs to the slot's rig and
                            // would push this head a fifth of a metre too high.
                            entity.headIsBase = false;
                        }
                        put(BoneHips, figure.hips);
                        put(BoneSpine, figure.spine);
                        put(BoneChest, figure.chest);
                        put(BoneNeck, figure.neck);
                        put(BoneHead, figure.head);
                        if (figure.limbs && !haveSlotFigure)
                        {
                            put(BoneLeftUpperLeg, figure.legL[0]);
                            put(BoneLeftLowerLeg, figure.legL[1]);
                            put(BoneLeftFoot, figure.legL[2]);
                            put(BoneRightUpperLeg, figure.legR[0]);
                            put(BoneRightLowerLeg, figure.legR[1]);
                            put(BoneRightFoot, figure.legR[2]);
                            put(BoneLeftUpperArm, figure.armL[0]);
                            put(BoneLeftLowerArm, figure.armL[1]);
                            put(BoneLeftHand, figure.armL[2]);
                            put(BoneRightUpperArm, figure.armR[0]);
                            put(BoneRightLowerArm, figure.armR[1]);
                            put(BoneRightHand, figure.armR[2]);
                        }
                    }
                    else if (!haveSlotFigure)
                    {
                        ++poseFail;
                        // Neither source answered for this body: carry the last
                        // measured figure. The bones are offsets from the anchor,
                        // so they still ride the body. 5 s, because a character
                        // sitting in a vehicle publishes no rows at all while the
                        // world around it keeps matching - the figure has to
                        // survive that stretch instead of blinking off for every
                        // passenger. The history is carried with it, otherwise the
                        // blend restarts from a stale pair and the first frame
                        // after the gap jumps.
                        const ULONGLONG nowUs = NowMicroseconds();
                        if (previousEntity != nullptr && nowUs > previousEntity->boneTick &&
                            nowUs - previousEntity->boneTick <= 5000000)
                        {
                            carryPreviousFigure(*previousEntity);
                        }
                    }
                }
                // Legacy path (model graph off): the slot's own rig was dropped
                // by BonePass - a seating jump moves every joint out of the
                // standing envelope - and SyncBonesFromSlots() skips a slot with
                // no hits, so nothing else refreshes these bones. Carry the last
                // measured figure the same way the pose path does.
                else if (g_options.skeleton && !usePose && !haveSlotFigure &&
                         previousEntity != nullptr)
                {
                    carryPreviousFigure(*previousEntity);
                }
                MarkStage("stage: pose sample done");
                CrownFromBase(entity.bones, entity.boneVisible, entity.headIsBase);
                AnatomicalCrown(entity.bones, entity.boneVisible);
                // Real head bone when the skeleton resolved: the box top then
                // follows crouch, prone and death poses instead of a constant
                // offset above the feet.
                if (entity.boneVisible[BoneHead])
                {
                    entity.head = Vec3{ entity.feet.x + entity.bones[BoneHead].x,
                                        entity.feet.y + entity.bones[BoneHead].y,
                                        entity.feet.z + entity.bones[BoneHead].z };
                }
                entity.predictedHead = entity.head;

                entity.sampleTick = NowMicroseconds();

                History *history = FindHistory(entity.key);
                if (history == nullptr)
                {
                    g_history.push_back(History{});
                    history = &g_history.back();
                    history->key = entity.key;
                }
                // Both follow filters keep their state in History, keyed on this
                // same component, so they survive this rebuild without copying
                // anything from the previous Entity.
                // The live: trace is gated to one line a second per body. That
                // gate lived on a field of a struct that Sample() rebuilds from
                // scratch, so it was born at 0 on every read and passed every
                // time: 33803 lines in two minutes, each one a CreateFileW +
                // WriteFile + CloseHandle from the present thread. The gate is
                // only a gate if the stamp survives the rebuild.
                if (previousEntity != nullptr)
                {
                    entity.traceTick = previousEntity->traceTick;
                }
                if (!UpdateKinematics(*history, entity.feet, NowMicroseconds()))
                {
                    // Teleport/respawn: the smoothed anchor belongs to the old
                    // position, so the next frame must snap instead of easing
                    // across the map. The joints ride the same teleport, so the
                    // joint follow restarts too.
                    history->filter.smoothInit = false;
                    history->filter.bonesSmoothValid = false;
                    ResetBoneHistory(history->filter);
                }
                // Millisecond stamp for the prediction freshness gate; the
                // velocity itself is measured on the QPC stamp above.
                history->tick = now;
                entity.velocity = history->velocity;
                entity.speed = aim::Length(history->velocity);
                if (!aim::Finite(entity.speed))
                {
                    entity.speed = 0.0f;
                }
                std::snprintf(entity.name, sizeof(entity.name), "%s", name);
                entity.id = static_cast<int>(g_entities.size()) + 1;

                // Hard cap, and the buffer is reserved once in StartWorker: a
                // reallocation here would free the array the render thread is
                // iterating this very frame, because the two run on different
                // threads now.
                if (g_entities.size() < 250)
                {
                    g_entities.push_back(entity);
                }
            }

            PruneHistory();

            if (g_entities.empty() && !sPrevious.empty() && !g_slots.empty())
            {
                g_entities = sPrevious;
                g_debug[0] = '\0';
                std::snprintf(g_debug, sizeof(g_debug), "sample: kept %u",
                              static_cast<unsigned>(g_entities.size()));
                Reproject(displayWidth, displayHeight);
                return;
            }

            // Баг 1.5: убран первый Reproject() из Sample() в Present-режиме.
            // Раньше вызывался здесь (до BonePass) и снова в ManagedTick
            // после SyncBonesFromSlots — двойной сдвиг smoothFeet за один
            // кадр давал постоянную микродрожь. Единственный Reproject теперь
            // только в ManagedTick после SyncBonesFromSlots().
            // MatchScreenPlates перенесён в конец Reproject() — там Entity уже
            // имеют правильные visible=true и boxTop, иначе он всегда давал 0.

            // Both numbers come from the frame Reproject finished, not from a
            // walk over g_entities: Sample() has just rebuilt every Entity from
            // scratch, so visible and onScreen are false here and the local
            // count printed "0 projected, boneAge=0 ms" over a full screen.
            const size_t projected = g_lastProjected;
            size_t named = 0;
            for (const Entity &entity : g_entities)
            {
                if (entity.nickname[0] != '\0')
                {
                    ++named;
                }
            }

            // One line a second: the native projection is only trustworthy if
            // it agrees with Unity's own answer, so it is checked against
            // Camera.WorldToScreenPoint on the first body Reproject drew. The
            // probe is captured there because this function no longer sees a
            // projected entity.
            static ULONGLONG lastProjectionCheck = 0;
            if (now - lastProjectionCheck >= 1000 && g_probeValid &&
                m_worldToScreen != nullptr && g_camera != nullptr)
            {
                lastProjectionCheck = now;
                Vec3 value = g_probeFeet;
                void *boxed = Invoke1(m_worldToScreen, g_camera, &value);
                Vec3 screen{};
                if (boxed != nullptr && g_resolver->ReadVec3(boxed, screen))
                {
                    char text[128];
                    std::snprintf(text, sizeof(text), "proj: native=%.1f,%.1f unity=%.1f,%.1f",
                                  g_probeFeetX, g_probeFeetY, screen.x,
                                  displayHeight - screen.y);
                    Trace(text);
                }
            }

            Trace("sample: done");
            size_t rigs = 0;
            size_t joints = 0;
            float farthest = 0.0f;
            for (const Slot &slot : g_slots)
            {
                if (slot.boneHits > 0)
                {
                    ++rigs;
                    joints += static_cast<size_t>(slot.boneHits);
                }
            }
            for (const Entity &entity : g_entities)
            {
                if (entity.visible && entity.distance > farthest)
                {
                    farthest = entity.distance;
                }
            }
            // The same metric as near=, so the two are comparable: the gate is
            // the nearest body to the eye, and a "far" measured as projection
            // depth used to read smaller than "near" in the same line, which is
            // impossible and was the tell that the eye point was wrong.
            if (g_cameraPositionValid)
            {
                for (size_t i = 0; i < count; ++i)
                {
                    const float distance =
                        aim::Length(aim::Sub(sPoints[i].feet, g_cameraPosition));
                    if (aim::Finite(distance) && distance > farthest)
                    {
                        farthest = distance;
                    }
                }
            }
            // Age of the oldest pose still on screen, in milliseconds. This is
            // the number the "bones trail and then snap" report is about: it has
            // to stay near the capture interval, and a value in the hundreds
            // means BonePass is not reaching every body on every pass. Published
            // by Reproject for the same reason projected is.
            const unsigned long boneAge = static_cast<unsigned long>(g_lastBoneAgeMs);
            size_t roles = 0;
            size_t armors = 0;
            for (const Entity &entity : g_entities)
            {
                if (entity.role[0] != '\0')
                {
                    ++roles;
                }
                if (entity.armor >= 0)
                {
                    ++armors;
                }
            }
            static ULONGLONG lastPoseSampleLog = 0;
            if (g_log != nullptr && GetTickCount64() - lastPoseSampleLog >= 2000)
            {
                lastPoseSampleLog = GetTickCount64();
                char line[160];
                std::snprintf(line, sizeof(line),
                              "pose sample: ok=%zu fail=%zu of %zu, model=%zu, live=%d",
                              poseOk, poseFail, count, model::Count(), pose::Live() ? 1 : 0);
                g_log(line);
            }

            // Why a visible body has no skeleton. Every two seconds, one line per
            // slot that is on screen with nothing to draw: "no rig", "parked for
            // N ms" and "rig dropped" are three different bugs, and the old log
            // only ever showed the aggregate `joints=`, which is the sum over all
            // bodies and hides a single boneless player completely.
            static ULONGLONG lastRigMissLog = 0;
            if (g_log != nullptr && g_options.skeleton && GetTickCount64() - lastRigMissLog >= 2000)
            {
                lastRigMissLog = GetTickCount64();
                const ULONGLONG nowTick = GetTickCount64();
                for (const Slot &slot : g_slots)
                {
                    if (slot.boneHits > 0 || slot.selfView)
                    {
                        continue;
                    }
                    if (!SlotNearViewport(slot, displayWidth, displayHeight))
                    {
                        continue;
                    }
                    char line[256];
                    const unsigned long long wait =
                        slot.rigCooldown > nowTick ? slot.rigCooldown - nowTick : 0ull;
                    const int rootMatch = (slot.charRoot != nullptr && slot.rigRoot != nullptr &&
                                           RigUnderTransform(slot.charRoot, slot.rigRoot))
                                              ? 1
                                              : 0;
                    std::snprintf(line, sizeof(line),
                                  "rig_miss: name=%s resolved=%d hits=%d cd=%llu ms faults=%u "
                                  "anim=%d rigRoot=%d charRoot=%d rootMatch=%d hipsDist=%.2f "
                                  "active=%d",
                                  slot.name[0] != '\0' ? slot.name : "?", slot.bonesResolved ? 1 : 0,
                                  slot.boneHits, wait, static_cast<unsigned>(slot.rigFaults),
                                  slot.animator != nullptr ? 1 : 0, slot.rigRoot != nullptr ? 1 : 0,
                                  slot.charRoot != nullptr ? 1 : 0, rootMatch,
                                  static_cast<double>(slot.hipsDist), Alive(slot.transform) ? 1 : 0);
                    g_log(line);
                }
            }

            std::snprintf(g_debug, sizeof(g_debug),
                          "walk: %zu found, %zu projected, %zu posfail, self=%d chain=%d/%zu gate=%d exact=%d, "
                          "near=%.1f, far=%.1f, boneAge=%lu ms, dm=%.2f, anch=%.2f, rigs=%zu/%zu, joints=%zu, skel=%d/%d, plates=%zu, named=%zu, "
                          "model=%zu roles=%zu armor=%zu, pos=%s, mtx=%s [%s]",
                          g_slots.size(), projected, failed, selfDropped ? 1 : 0, g_selfByChain,
                          g_cameraChain.size(), g_cameraPositionValid ? 1 : 0,
                          exactSelf >= 0 ? 1 : 0, nearest, farthest,
                          static_cast<unsigned long>(boneAge), g_drawnArmJumpM, g_anchorDelta,
                          rigs,
                          g_slots.size(), joints, g_options.skeleton ? 1 : 0,
                          static_cast<int>(g_bonePasses), g_plates.size(), named, model::Count(),
                          roles, armors, g_positionNative ? "inj" : "box",
                          g_matrixNative ? "inj" : "box", g_anchorSource);

            // The next window starts from zero so dm= is the worst frame of the
            // last second, not a maximum that never comes down.
            g_drawnArmJumpM = 0.0f;
            g_anchorDelta = 0.0f;

            // Into the log whenever the line changes, plus a slow heartbeat, so a
            // run in a match is diagnosable from the file alone: a silent run
            // after this point means the process died, not that the walk failed.
            static char lastLogged[320] = "";
            static ULONGLONG lastLogTick = 0;
            const bool changed = std::strcmp(g_debug, lastLogged) != 0;
            if (g_log != nullptr && (changed ? now - lastLogTick > 2000 : now - lastLogTick > 15000))
            {
                lastLogTick = now;
                std::snprintf(lastLogged, sizeof(lastLogged), "%s", g_debug);
                g_log(g_debug);
            }
        }

        // The lead is applied every frame, not only when the sample runs. Only
        // the world point is computed here; the pixel lead comes out of the
        // per-frame projection, so it stays correct while the camera turns.
        // The target list is passed in: in worker mode the render thread must
        // not iterate the worker's g_entities (or its history) while the worker
        // is rewriting it. Velocity and its sample stamp travel inside the
        // published snapshot, so no history lookup is needed here at all.
        void RefreshLead(std::vector<Entity> &entities)
        {
            const float leadSeconds = LeadSeconds();
            const ULONGLONG nowUs = NowMicroseconds();

            for (Entity &entity : entities)
            {
                Vec3 predicted = entity.head;
                const bool velocityFresh =
                    entity.sampleTick != 0 && nowUs >= entity.sampleTick &&
                    (nowUs - entity.sampleTick) <= kVelocityDropMs * 1000ull;
                if (leadSeconds > 0.0f && velocityFresh)
                {
                    predicted = aim::Predict(entity.head, entity.velocity, leadSeconds, 0.0f);
                }
                entity.predictedHead = predicted;
                if (leadSeconds <= 0.0f)
                {
                    entity.leadX = 0.0f;
                    entity.leadY = 0.0f;
                    entity.predictedHeadX = entity.headX;
                    entity.predictedHeadY = entity.headY;
                }
            }
        }
    }

    void Attach(Il2CppResolver *resolver, void (*log)(const char *message))
    {
        g_resolver = resolver;
        g_log = log;
        model::Attach(resolver, log);
        pose::Attach(resolver, log);
    }

    Options &Config() { return g_options; }
    char *TargetClassName() { return g_className; }
    char *TargetNamespace() { return g_classNamespace; }
    float &HeadOffset() { return g_headOffset; }

    // Manual rescan clears the auto-disable: Olivia pressing the button is the
    // one case where retrying a faulty scan is worth the risk of a hang.
    void Rescan()
    {
        g_scanFaults = 0;
        g_scanCooldownUntil = 0;
        g_forceScan = true;
    }

    void Hold(unsigned int milliseconds)
    {
        g_holdUntil = GetTickCount64() + milliseconds;
    }

    // --- static TLS for threads the manual map never told ntdll about -------
    // A loader-mapped image gets its TLS slot from ntdll, which also fills the
    // block on every thread. A manually mapped image does not: our PE carries a
    // TLS directory of exactly 8 bytes (the two __cxa_eh_globals pointers), the
    // loader says it out loud ("8 bytes of static TLS left uninitialised"), and
    // any thread that runs C++ with unwind info reads a garbage pointer there.
    // That is the fault signature we chased for weeks: rip = "<private block> +
    // 0xED7", module=unmapped, dead 1-2 s after install, while every run that
    // avoided C++ with exception frames stayed alive for minutes.
    //
    // What follows gives the current thread a real block: take the image's own
    // slot (or allocate one if the image has none), allocate a zeroed copy of
    // the initial data, hand it to the thread, and nothing else. The image's TLS
    // callbacks are NOT called — MinGW's is a module initialiser that has
    // already run at map time, and re-entering it on a thread attach is more
    // risk than the zeroed block is worth.
    struct TlsSetup
    {
        unsigned long index = 0;
        const unsigned char *initial = nullptr;
        size_t rawBytes = 0;
        size_t blockBytes = 0;
    };

    constexpr LONG kTlsIdle = 0;
    constexpr LONG kTlsBusy = 1;
    constexpr LONG kTlsReady = 2;
    constexpr LONG kTlsImpossible = 3;

    TlsSetup g_tls{};
    volatile LONG g_tlsState = kTlsIdle;

    bool InsideImage(unsigned long long value, unsigned long long base,
                     unsigned long long size)
    {
        return value >= base && value < base + size;
    }

    // The TLS directory holds the addresses the image was LINKED for, and a
    // manual map moves the image without fixing that directory up, so every
    // pointer in it is re-based against the recorded ImageBase first; the raw
    // value is kept as a fallback so a mapper that did fix it up still works.
    const void *RebasePointer(unsigned long long value, unsigned long long base,
                              unsigned long long size, unsigned long long linkedBase)
    {
        if (value == 0)
        {
            return nullptr;
        }
        const unsigned long long moved = base + (value - linkedBase);
        if (InsideImage(moved, base, size))
        {
            return reinterpret_cast<const void *>(moved);
        }
        if (InsideImage(value, base, size))
        {
            return reinterpret_cast<const void *>(value);
        }
        return nullptr;
    }

    const unsigned char *FindOwnImage()
    {
        unsigned long long candidate =
            reinterpret_cast<unsigned long long>(&FindOwnImage) & ~0xFFFFull;
        for (int step = 0; step < 0x40 && candidate >= 0x10000;
             ++step, candidate -= 0x10000)
        {
            MEMORY_BASIC_INFORMATION info{};
            if (VirtualQuery(reinterpret_cast<LPCVOID>(candidate), &info, sizeof(info)) == 0 ||
                info.State != MEM_COMMIT || (info.Protect & (PAGE_NOACCESS | PAGE_GUARD)) != 0)
            {
                continue;
            }
            const auto *dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(candidate);
            if (dos->e_magic != IMAGE_DOS_SIGNATURE || dos->e_lfanew <= 0 ||
                dos->e_lfanew > 0x1000)
            {
                continue;
            }
            const auto *nt = reinterpret_cast<const IMAGE_NT_HEADERS64 *>(
                candidate + static_cast<unsigned long>(dos->e_lfanew));
            if (nt->Signature != IMAGE_NT_SIGNATURE)
            {
                continue;
            }
            return reinterpret_cast<const unsigned char *>(candidate);
        }
        return nullptr;
    }

    bool PrepareTls()
    {
        // Bisect switch, no rebuild: an empty %TEMP%\OmegaCore_notls skips the
        // per-thread TLS block entirely, which is the shape the 18:32 build had
        // before this code existed. The crash after it appeared landed on
        // _M_construct (RVA 0xED7 = the call to _M_create), i.e. exactly the
        // code that touches the static TLS pointer, so the switch decides
        // whether the block is the cause.
        {
            wchar_t marker[MAX_PATH]{};
            if (GetTempPathW(MAX_PATH, marker) != 0)
            {
                lstrcatW(marker, L"OmegaCore_notls");
                if (GetFileAttributesW(marker) != INVALID_FILE_ATTRIBUTES)
                {
                    Trace("tls: off (sentinel)");
                    return false;
                }
            }
        }

        const unsigned char *base = FindOwnImage();
        if (base == nullptr)
        {
            return false;
        }
        const auto *dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(base);
        const auto *nt = reinterpret_cast<const IMAGE_NT_HEADERS64 *>(
            base + static_cast<unsigned long>(dos->e_lfanew));
        const IMAGE_DATA_DIRECTORY &dir =
            nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_TLS];
        if (dir.VirtualAddress == 0 || dir.Size < sizeof(IMAGE_TLS_DIRECTORY64))
        {
            return false;
        }
        const unsigned long long linkedBase = nt->OptionalHeader.ImageBase;
        const unsigned long long size = nt->OptionalHeader.SizeOfImage;
        const unsigned long long imageBase = reinterpret_cast<unsigned long long>(base);
        // A data directory entry is an RVA, so this one needs no re-basing.
        const auto *tls = reinterpret_cast<const IMAGE_TLS_DIRECTORY64 *>(
            base + dir.VirtualAddress);

        const unsigned long long start = tls->StartAddressOfRawData;
        const unsigned long long end = tls->EndAddressOfRawData;
        const size_t rawBytes = end > start ? static_cast<size_t>(end - start) : 0;
        const size_t blockBytes = rawBytes + static_cast<size_t>(tls->SizeOfZeroFill);
        if (blockBytes == 0)
        {
            return false;
        }

        auto *indexSlot = reinterpret_cast<unsigned long *>(const_cast<void *>(
            RebasePointer(tls->AddressOfIndex, imageBase, size, linkedBase)));
        unsigned long index = 0;
        if (indexSlot != nullptr && *indexSlot != 0 && *indexSlot < 1088)
        {
            index = *indexSlot;  // ntdll already handed this image a slot
        }
        else
        {
            index = TlsAlloc();
            if (index == TLS_OUT_OF_INDEXES)
            {
                return false;
            }
            if (indexSlot != nullptr)
            {
                *indexSlot = index;
            }
        }

        g_tls.index = index;
        g_tls.initial = static_cast<const unsigned char *>(
            RebasePointer(start, imageBase, size, linkedBase));
        g_tls.rawBytes = g_tls.initial != nullptr ? rawBytes : 0;
        g_tls.blockBytes = blockBytes;

        char line[128];
        std::snprintf(line, sizeof(line), "tls: slot %lu, %llu+%lu bytes, image %p",
                      index, static_cast<unsigned long long>(blockBytes),
                      static_cast<unsigned long>(tls->SizeOfZeroFill),
                      static_cast<const void *>(base));
        Trace(line);
        return true;
    }

    void EnsureThreadTls()
    {
        if (g_tlsState != kTlsReady)
        {
            if (g_tlsState == kTlsImpossible)
            {
                return;
            }
            if (InterlockedCompareExchange(&g_tlsState, kTlsBusy, kTlsIdle) == kTlsIdle)
            {
                const bool ok = PrepareTls();
                InterlockedExchange(&g_tlsState, ok ? kTlsReady : kTlsImpossible);
                if (!ok)
                {
                    Trace("tls: no directory, running without a block");
                    return;
                }
            }
            else
            {
                return;  // another thread is preparing it, retry next call
            }
        }

        if (TlsGetValue(g_tls.index) != nullptr)
        {
            return;
        }
        auto *block = static_cast<unsigned char *>(
            VirtualAlloc(nullptr, g_tls.blockBytes, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
        if (block == nullptr)
        {
            return;
        }
        for (size_t i = 0; i < g_tls.rawBytes; ++i)
        {
            block[i] = g_tls.initial[i];
        }
        TlsSetValue(g_tls.index, block);
        Trace("tls: block attached to this thread");
    }

    // --- publish to render thread -------------------------------------------
    // Copies the worker's entity list and MVP into a shared snapshot under an
    // exclusive SRWLOCK.  The render thread picks it up under a shared lock,
    // so the swap itself is sub-microsecond.  Called at the end of every
    // ManagedTick path that modifies g_entities or g_mvp.
    void PublishEntities()
    {
        AcquireSRWLockExclusive(&g_entityLock);
        g_publishedEntities = g_entities;
        g_publishedPlates   = g_plates;
        std::memcpy(g_publishedMvp, g_mvp, sizeof(g_mvp));
        g_publishedMvpValid = g_mvpValid;
        // The version moves together with the data it describes, under the same
        // lock: the reader compares it outside the lock to skip a copy on a
        // frame where nothing was published.
        InterlockedIncrement64(&g_publishSeq);
        ReleaseSRWLockExclusive(&g_entityLock);
    }

    // Called by the render thread to grab the latest snapshot.  Copies under a
    // shared lock (sub-microsecond), then the render thread works on its own
    // g_renderEntities without holding any lock.
    //
    // The copy is skipped when the version did not move: Reproject writes the
    // screen-space result and both follow filters into g_renderEntities every
    // frame, and re-copying an unchanged snapshot threw that away and restarted
    // the follows. The filters themselves live in g_renderFilters now - the
    // render thread never reads g_history, which the worker reallocates.
    void FetchEntities()
    {
        if (InterlockedCompareExchange64(&g_publishSeq, 0, 0) == g_renderSeq)
        {
            return;
        }

        AcquireSRWLockShared(&g_entityLock);
        g_renderEntities = g_publishedEntities;
        g_renderPlates   = g_publishedPlates;
        std::memcpy(g_renderMvp, g_publishedMvp, sizeof(g_renderMvp));
        g_renderMvpValid = g_publishedMvpValid;
        g_renderSeq = g_publishSeq;
        ReleaseSRWLockShared(&g_entityLock);

        // Filters of bodies that left the snapshot are dead weight and, worse,
        // would be applied to the next character reusing that pooled pointer.
        // Swap-and-pop: the order of this list means nothing.
        for (size_t i = 0; i < g_renderFilters.size();)
        {
            bool live = false;
            for (const Entity &entity : g_renderEntities)
            {
                if (entity.key == g_renderFilters[i].key)
                {
                    live = true;
                    break;
                }
            }
            if (live)
            {
                ++i;
            }
            else
            {
                g_renderFilters[i] = g_renderFilters.back();
                g_renderFilters.pop_back();
            }
        }
    }

    // --- thread split -------------------------------------------------------
    // Every il2cpp call now runs on a Windows thread-pool thread, never on the
    // game's render thread. Evidence that this was the whole problem: pid 24408
    // hung with the trace frozen inside "scan: find" (responding=False), pid
    // 18312 died one line after "fault: at scan: find", and the runs with no
    // managed work at all stayed alive at 60 us per frame. A pool thread has a
    // system start address (TppWorkerThread), so it does not look like a thread
    // created out of a manually mapped image, which is what EAC took the process
    // down for earlier.
    float g_displayWidth = 0.0f;
    float g_displayHeight = 0.0f;

    // Phase split (step 1 of the night spec). A capture is not run inside
    // Tick() any more: Tick() only arms this flag, and TickAfterPresent() runs
    // Sample + BonePass + SyncBonesFromSlots right after the game's own
    // Present returned. Inside the frame the rig is read between the base
    // animation clip and the weapon IK pass, and a burst of 2-4 such reads
    // became the norm for the filter - the fixed-step 0.33-0.49 m jump on
    // standing armed bodies. After Present the frame is whole.
    bool g_capturePending = false;
    float g_captureWidth = 0.0f;
    float g_captureHeight = 0.0f;

    // Bisect switch for the method itself: an empty file OmegaCore_present_esp
    // in %TEMP% puts the il2cpp side back on the game's Present thread - the
    // method that ran clean for minutes at 18:32 (build 1829763, walk 7-10,
    // rigs 7/7) before the pool worker and the thread TLS block existed. EAC
    // flagged 0xC0000001 on the worker build; this switch separates the two
    // without a rebuild.
    bool SentinelPresent(const wchar_t *name)
    {
        wchar_t path[MAX_PATH]{};
        if (GetTempPathW(MAX_PATH, path) == 0)
        {
            return false;
        }
        lstrcatW(path, name);
        return GetFileAttributesW(path) != INVALID_FILE_ATTRIBUTES;
    }

    // The joints are read by BonePass(), which runs AFTER Sample() on purpose:
    // it stamps poseFeet from the feet that are current at read time. That means
    // the joints in g_slots are one half-tick fresher than the copy Sample()
    // pushed into g_entities, and without this the figure lags the body by a
    // whole tick. Copies each slot's capture into its entity in the same
    // semantics Sample() uses: offsets from the read anchor, per-joint validity,
    // and the thigh-midpoint fork for the hips this rig does not expose.
    void SyncBonesFromSlots()
    {
        for (Entity &entity : g_entities)
        {
            if (entity.key == nullptr)
            {
                continue;
            }
            // A body whose figure came from the shared hitbox table carries no
            // slot capture (boneTick 0 on the first tick it is seen). Skipping
            // those bodies was the reason a freshly appeared character kept a
            // seven-point torso from the table for an entire session while its
            // 19 measured joints sat unused in the slot: the sync only ever
            // advanced a capture that already existed, and nothing created the
            // first one. The stamp comparison below is the real guard - a slot
            // with no fresh capture (poseTick 0) is skipped, and an unchanged
            // poseTick is not copied twice.
            bool synced = false;
            for (const Slot &slot : g_slots)
            {
                if (slot.component != entity.key || slot.boneHits <= 0 || slot.poseTick == 0)
                {
                    continue;
                }
                if (slot.poseTick != entity.boneTick)
                {
                    entity.bonePrevTick = entity.boneTick;
                    entity.bonesPrevValid = (entity.boneTick != 0);
                    for (int bone = 0; bone < kBoneCount; ++bone)
                    {
                        entity.bonesPrev[bone] = entity.bones[bone];
                    }

                    // Step 2 of the spec: the anchor can change between "feet"
                    // and "hips" when a body sits in a car or steps out. The
                    // offsets then change meaning (hips ~ +0.90 m above the feet
                    // standing, ~0.00 seated), so a mix of the two in one medoid
                    // window reads as a 0.9 m jump on every joint: the filter
                    // holds the standing pose and the follow then snaps to the
                    // seated one. A change of more than 0.45 m in the hips height
                    // is that switch, and it resets the smoothing instead of
                    // averaging across it.
                    const Vec3 previousHips = entity.bones[BoneHips];
                    const bool havePreviousHips =
                        entity.boneTick != 0 && entity.boneVisible[BoneHips];
                    entity.boneTick = slot.poseTick;
                    entity.poseAnchor = (slot.haveFeet && slot.poseTick != 0)
                                            ? slot.poseFeet
                                            : aim::Add(entity.feet, entity.anchorOffset);
                    for (int bone = 0; bone < kBoneCount; ++bone)
                    {
                        entity.bones[bone] = aim::Sub(slot.bonePos[bone], entity.poseAnchor);
                        entity.boneVisible[bone] = slot.boneOk[bone];
                    }
                    if (havePreviousHips && entity.boneVisible[BoneHips] &&
                        std::fabs(entity.bones[BoneHips].y - previousHips.y) > 0.45f)
                    {
                        entity.bonesPrevValid = false;
                        for (int bone = 0; bone < kBoneCount; ++bone)
                        {
                            entity.bonesPrev[bone] = entity.bones[bone];
                        }
                        if (History *history = FindHistory(entity.key))
                        {
                            ResetBoneHistory(history->filter);
                        }
                        static ULONGLONG anchorSwitchAt = 0;
                        const ULONGLONG anchorNow = GetTickCount64();
                        if (anchorNow - anchorSwitchAt >= 2000)
                        {
                            anchorSwitchAt = anchorNow;
                            char switchLine[192];
                            std::snprintf(switchLine, sizeof(switchLine),
                                          "bones: anchor mode changed (hips %.2f -> %.2f m), "
                                          "smoothing reset",
                                          previousHips.y, entity.bones[BoneHips].y);
                            TraceLine(switchLine);
                        }
                    }
                    entity.headIsBase = slot.headIsBase;
                    if (!entity.boneVisible[BoneHips] && entity.boneVisible[BoneLeftUpperLeg] &&
                        entity.boneVisible[BoneRightUpperLeg])
                    {
                        const Vec3 &thighLeft = entity.bones[BoneLeftUpperLeg];
                        const Vec3 &thighRight = entity.bones[BoneRightUpperLeg];
                        entity.bones[BoneHips] = Vec3{ (thighLeft.x + thighRight.x) * 0.5f,
                                                       (thighLeft.y + thighRight.y) * 0.5f,
                                                       (thighLeft.z + thighRight.z) * 0.5f };
                        entity.boneVisible[BoneHips] = true;
                    }
                    synced = true;
                }
                break;
            }
            // 1.5: the anatomical crown clamp runs on the joints that are
            // actually drawn. BonePass stamps a fresh capture and Sample()
            // clamps two frames later, so between them the raw rig head node
            // (a head-height above the skull on some skins) was drawn
            // unclamped. Same rule, applied where the bones land.
            if (synced)
            {
                CrownFromBase(entity.bones, entity.boneVisible, entity.headIsBase);
                AnatomicalCrown(entity.bones, entity.boneVisible);
            }
        }
    }

    void ManagedTick(float displayWidth, float displayHeight);

    DWORD WINAPI WorkerProc(LPVOID)
    {
        Trace("worker: proc entered");
        // Before anything else: this thread was not created by the loader, so it
        // has no static TLS block for this image. Without it the first C++ frame
        // with unwind info dereferences garbage and the process dies inside a
        // private allocation at +0xED7.
        EnsureThreadTls();
        // The worker samples at 8 ms. Sleep(10) on the default 15.6 ms timer
        // resolution quantized the loop to ~16 ms, so the configured rate was
        // never reached and every position arrived late. 1 ms resolution plus a
        // 2 ms sleep gives the loop its own tick instead of the system's.
        timeBeginPeriod(1);
        {
            char identity[96];
            std::snprintf(identity, sizeof(identity), "worker: loop start tid=%lu",
                          static_cast<unsigned long>(GetCurrentThreadId()));
            Trace(identity);
        }
        for (;;)
        {
            const float width = g_displayWidth;
            const float height = g_displayHeight;
            if (width > 1.0f && height > 1.0f)
            {
                ManagedTick(width, height);
            }
            Sleep(2);
        }
        return 0;
    }

    // Evidence only: no managed call, no shared state, one line per second.
    DWORD WINAPI WatchdogProc(LPVOID)
    {
        EnsureThreadTls();
        LONG64 lastSeq = -1;
        ULONGLONG stallSince = GetTickCount64();
        for (;;)
        {
            const LONG64 seq = g_frameSeq;
            const ULONGLONG nowTick = GetTickCount64();
            if (seq != lastSeq)
            {
                lastSeq = seq;
                stallSince = nowTick;
            }
            else if (nowTick - stallSince >= 1500 && g_hangDumped != seq && seq > 2)
            {
                // Exactly one dump per stall: the render thread is wedged and
                // every further snapshot would say the same thing.
                g_hangDumped = seq;
                DumpHang("render thread stopped");
            }
            char text[64];
            std::snprintf(text, sizeof(text), "wdg: tick=%lu frames=%lld",
                          static_cast<unsigned long>(nowTick & 0xFFFFFFFFu),
                          static_cast<long long>(seq));
            TraceLine(text);
            Sleep(1000);
        }
        return 0;
    }

    // Starts the watchdog only: sentinel-gated, one line per second, no managed
    // call. If the script file keeps growing while `beat.raw` stops, the render
    // thread is stuck and the process is healthy; if it stops too, the whole
    // runtime stopped (Boehm stop-the-world) and the present hook is innocent.
    void StartWatchdog()
    {
        static LONG started = 0;
        if (started != 0 || !SentinelPresent(L"OmegaCore_wdg"))
        {
            return;
        }
        if (InterlockedCompareExchange(&started, 1, 0) != 0)
        {
            return;
        }
        if (QueueUserWorkItem(WatchdogProc, nullptr, 0) == FALSE)
        {
            Trace("wdg: queue failed");
        }
    }

    void StartWorker()
    {
        static LONG resolved = 0;
        if (InterlockedCompareExchange(&resolved, 1, 0) == 0)
        {
            if (SentinelPresent(L"OmegaCore_present_esp"))
            {
                g_workerMode = false;
                Trace("worker: off (sentinel), esp on the present thread");
            }
            // Reserved before the render thread ever looks at the list: the
            // producer rewrites it, and a reallocation mid-frame would free the
            // array under the drawing loop.
            g_entities.reserve(256);
            g_publishedEntities.reserve(256);
            g_renderEntities.reserve(256);
        }
        if (!g_workerMode)
        {
            return;
        }
        static LONG started = 0;
        if (InterlockedCompareExchange(&started, 1, 0) != 0)
        {
            return;
        }
        Trace("worker: queued on the system pool");
        // The queue result was never checked, and the last live run had
        // "worker: queued" with no "worker: loop start" after it: either the
        // callback never ran or it died before its first line.
        if (QueueUserWorkItem(WorkerProc, nullptr, 0) == FALSE)
        {
            char text[96];
            std::snprintf(text, sizeof(text), "worker: queue failed, win32 %lu",
                          static_cast<unsigned long>(GetLastError()));
            Trace(text);
        }
    }

    void Tick(float displayWidth, float displayHeight)
    {
        // The render thread is a game thread, not a loader thread for this image,
        // so it needs the same block the worker gets (see EnsureThreadTls).
        EnsureThreadTls();
        StartWatchdog();
        {
            static bool renderIdentityLogged = false;
            if (!renderIdentityLogged)
            {
                renderIdentityLogged = true;
                char identity[96];
                std::snprintf(identity, sizeof(identity), "worker: render thread tid=%lu",
                              static_cast<unsigned long>(GetCurrentThreadId()));
                Trace(identity);
                // The pose blend is arithmetic that no log line can prove, so it
                // is proven against closed forms once and reported like the other
                // self checks.
                char check[64];
                std::snprintf(check, sizeof(check), "pose blend self check : %s (case %d)",
                              blend::SelfCheck(kPoseMaxStep) == 0 ? "pass" : "FAIL",
                              blend::SelfCheck(kPoseMaxStep));
                Trace(check);
            }
        }
        g_displayWidth = displayWidth;
        g_displayHeight = displayHeight;
        if (!g_workerMode)
        {
            ManagedTick(displayWidth, displayHeight);
            return;
        }
        if (displayWidth <= 1.0f || displayHeight <= 1.0f)
        {
            return;
        }
        // Render thread: projection and drawing only. Matrices come from the
        // worker's last refresh (a few milliseconds old at most), so nothing
        // here touches il2cpp and nothing here can hitch the game.
        // Fetch the latest published snapshot from the worker. The copy runs
        // under a shared SRWLOCK (sub-microsecond); once released, the render
        // thread works on its own g_renderEntities — no lock held, no race.
        FetchEntities();
        RefreshLead(g_renderEntities);
        Reproject(displayWidth, displayHeight, false, &g_renderEntities);
    }

    void ManagedTick(float displayWidth, float displayHeight)
    {
        if (!g_options.enabled && !g_options.aimEnabled)
        {
            g_entities.clear();
            PublishEntities();
            return;
        }
        if (displayHeight <= 1.0f || displayWidth <= 1.0f)
        {
            return;
        }

        const ULONGLONG now = GetTickCount64();
        // One-shot breadcrumbs inside the tick. The last two deaths left the raw
        // trace at "stage: tick start" with nothing after it, which says the tick
        // never returned but not which call it was in. These cost nothing and
        // turn the next death into a line number.
        {
            static bool enterLogged = false;
            if (!enterLogged)
            {
                enterLogged = true;
                Trace("tick: enter");
            }
        }
        // Nothing managed runs until the runtime is actually up. The Present
        // thread is a foreign thread for il2cpp, and attaching it while the game
        // is still booting faults on the FIRST managed call — every crash so far
        // landed one second after install, with rip 0x...0ED7, module=unmapped,
        // and nothing after "stage: tick start". The only honest signal that
        // costs no managed call is the age of the process: the game is past its
        // loading screens well before this, and a late inject is unaffected.
        {
            // The start time is read once; the age is recomputed every frame.
            // Caching the AGE froze the gate shut: a process 38 s old at inject
            // time stayed "38 s" forever and Tick returned on every frame —
            // "ESP 0 entities / no walk yet" while the menu was open in a live
            // world (pid 24340, 20:46:41: "runtime warmup (process 38562 ms,
            // need 90000)" once, then nothing, cost mean 1 us).
            static ULARGE_INTEGER processStarted{};
            if (processStarted.QuadPart == 0)
            {
                FILETIME creation{};
                FILETIME exit{};
                FILETIME kernel{};
                FILETIME user{};
                if (GetProcessTimes(GetCurrentProcess(), &creation, &exit, &kernel, &user))
                {
                    processStarted.LowPart = creation.dwLowDateTime;
                    processStarted.HighPart = creation.dwHighDateTime;
                }
            }
            ULONGLONG processAgeMs = 0;
            if (processStarted.QuadPart != 0)
            {
                ULARGE_INTEGER clock{};
                FILETIME nowFile{};
                GetSystemTimeAsFileTime(&nowFile);
                clock.LowPart = nowFile.dwLowDateTime;
                clock.HighPart = nowFile.dwHighDateTime;
                if (clock.QuadPart > processStarted.QuadPart)
                {
                    processAgeMs = (clock.QuadPart - processStarted.QuadPart) / 10000ull;
                }
            }
            constexpr ULONGLONG kRuntimeWarmupMs = 15000;
            if (processAgeMs != 0 && processAgeMs < kRuntimeWarmupMs)
            {
                static bool warmupLogged = false;
                if (!warmupLogged)
                {
                    warmupLogged = true;
                    char text[128];
                    std::snprintf(text, sizeof(text),
                                  "tick: runtime warmup (process %llu ms, need %llu)",
                                  static_cast<unsigned long long>(processAgeMs),
                                  static_cast<unsigned long long>(kRuntimeWarmupMs));
                    TraceLine(text);
                }
                return;
            }
        }
        // The second gate: a main camera. Injecting while the intro movie or a
        // loading screen is up put the scene walk (FindObjectsOfType) on top of a
        // half-built scene and took the whole process down one second after
        // install. No camera means no box could be drawn anyway, so this costs
        // nothing that was ever visible.
        static bool cameraSeen = false;
        static ULONGLONG cameraSince = 0;
        if (!cameraSeen)
        {
            // Metadata only: the resolver has to know Camera.get_main before it
            // can answer "is the world up". This is also why the gate has to sit
            // AFTER ResolveAnchor instead of before it — without it the method
            // pointer stayed null and the gate never opened, which is "ESP dead".
            if (!ResolveAnchor())
            {
                static bool anchorFailLogged = false;
                if (!anchorFailLogged)
                {
                    anchorFailLogged = true;
                    Trace("tick: anchor failed");
                }
                return;
            }
            static bool anchorOkLogged = false;
            if (!anchorOkLogged)
            {
                anchorOkLogged = true;
                Trace("tick: anchor ok, asking the camera");
            }
            cameraSeen = RefreshCamera();
            if (!cameraSeen)
            {
                static bool guardLogged = false;
                if (!guardLogged)
                {
                    guardLogged = true;
                    Trace("tick: waiting for the world (no main camera)");
                }
                return;
            }
            cameraSince = now;
            static bool cameraOkLogged = false;
            if (!cameraOkLogged)
            {
                cameraOkLogged = true;
                Trace("tick: camera ok, world is up");
            }
        }
        // The camera exists a beat before the scene graph is safe to enumerate:
        // a scan inside that window is the same fault, just harder to hit.
        constexpr ULONGLONG kWorldSettleMs = 500;
        if (cameraSince != 0 && now - cameraSince < kWorldSettleMs)
        {
            return;
        }

        // The model graph advances on plain memory reads, so it runs even while
        // the managed walk is standing down: that is what keeps boxes on screen
        // through a fault, a menu hold or a camera cut.
        // One-shot brackets: the run that crashed on the first present frame left
        // no trace at all, so "model enter" without "model ok" is the answer to
        // whether the new module is the one that faulted.
        static bool modelEnterLogged = false;
        if (!modelEnterLogged)
        {
            modelEnterLogged = true;
            TraceLine("tick: model enter");
        }
        model::Tick(now);
        MarkStage("stage: tick enter");
        // Ready() OR Requested(): while the graph is being located the list is
        // empty, and that is not a reason to fall back to FindObjectsOfType —
        // that managed call is what froze pid 19440 inside the scan.
        const bool modelMode = model::Ready() || model::Requested();
        // The pose module only exists to feed the model path. With the graph off
        // it ran anyway for five seconds per match, on the Present thread, and
        // only ever reached "pose: locating" -> "no SkeletonConfig".
        if (modelMode && (g_options.skeleton || pose::Ready()))
        {
            pose::Tick(now);
            MarkStage("stage: pose tick done");
        }
        static bool modelOkLogged = false;
        if (!modelOkLogged)
        {
            modelOkLogged = true;
            char modelLine[160];
            std::snprintf(modelLine, sizeof(modelLine), "tick: model ok | %s", model::Status());
            TraceLine(modelLine);
        }

        if (now < g_faultUntil || now < g_holdUntil)
        {
            // Keep what was already sampled: the world positions are still the
            // best information available, and dropping them for five seconds is
            // what made the esp blink out after a fault. Only the managed walk
            // stands down, the projection keeps tracking the camera. Which of the
            // two stands down is written down — "the esp froze" reports used to
            // have no way of telling a fault from a menu hold.
            static ULONGLONG lastStandDown = 0;
            if (now - lastStandDown >= 1000)
            {
                lastStandDown = now;
                Trace(now < g_faultUntil ? "tick: fault backoff" : "tick: hold");
            }
            RefreshLead(g_entities);
            // In worker mode the projection and the follows belong to the
            // render thread; running them here would race its reads of the
            // very list being published.
            if (!g_workerMode)
            {
                Reproject(displayWidth, displayHeight);
            }
            PublishEntities();
            return;
        }

        const bool anchorReady = ResolveAnchor();
        if (!anchorReady && !modelMode)
        {
            // The anchor is gone (scene change, class rename). The last
            // positions go stale, so they are dropped instead of being drawn in
            // the wrong place, and the next tick retries the resolution.
            g_entities.clear();
            PublishEntities();
            return;
        }

        const ULONGLONG scanInterval =
            static_cast<ULONGLONG>(std::clamp(g_options.scanIntervalMs, 500, 60000));
        // A forced rescan must still be rate limited. On a lobby change every
        // cached component is a freed object, the walk raises, g_forceScan stays
        // true and the next frame paid another FindObjectsOfType (25 ms measured)
        // — the whole game hitched for as long as the scene took to settle. One
        // scan per kForcedScanGapMs is enough to find the new scene.
        constexpr ULONGLONG kForcedScanGapMs = 1000;
        const bool forcedReady = g_lastScan == 0 || now - g_lastScan >= kForcedScanGapMs;
        // While the model graph is delivering characters there is nothing left to
        // discover by hand. FindObjectsOfType is a managed call on the render
        // thread, and it is the one that deadlocked the game (pid 24408: the log
        // stops inside "scan: find", 25 ms per walk, plates another 21 ms). The
        // model path reads the same characters straight out of memory.
        //
        // The one thing the graph cannot supply is the animated joint hierarchy:
        // BattleCharacterModel keeps SkeletonModel empty for remote characters
        // (the external client reads their pose out of the shared hitbox buffer,
        // 7 bones). So the view slots are still the only 19-joint source, and with
        // the walk switched off completely they never existed — pid 26236 drew
        // "0 found ... rigs=0/0, joints=0" for its whole life. The walk therefore
        // runs in model mode too, but only well after the graph is live (never
        // during the locate path that hung the game) and on a slow timer.
        constexpr ULONGLONG kModelScanSettleMs = 3000;
        constexpr ULONGLONG kModelScanIntervalMs = 20000;
        if (modelMode && g_modelReadySince == 0 && model::Ready())
        {
            g_modelReadySince = now;
            g_forceScan = true;
        }
        const bool modelScanDue =
            g_options.legacyScan && modelMode && g_modelReadySince != 0 &&
            now - g_modelReadySince >= kModelScanSettleMs &&
            (g_lastScan == 0 || now - g_lastScan >= kModelScanIntervalMs);
        const bool legacyScanDue =
            g_options.legacyScan && !modelMode &&
            ((g_forceScan && forcedReady) || g_lastScan == 0 || now - g_lastScan >= scanInterval);
        if (legacyScanDue || (modelScanDue && forcedReady))
        {
            // The exact self test (SelfByFollowCamera) reads the eye position,
            // and that is produced by RefreshMatrices - which used to run for the
            // first time in Sample(), AFTER this scan. On the first scan after a
            // start or a scene change it therefore always answered "no camera
            // position", the exact pass came back empty and the distance guess
            // picked the body. One matrix read here closes that window.
            if (!g_cameraPositionValid)
            {
                RefreshMatrices();
            }
            g_lastScan = now;
            const ULONGLONG scanStarted = NowMicroseconds();
            Scan();
            // The scan runs managed calls on the Present thread, so it costs
            // frames. Olivia reported a single blink every few seconds: this line
            // measures it instead of guessing, and the interval is the same value
            // drawn in the menu.
            const ULONGLONG scanCost = NowMicroseconds() - scanStarted;
            if (scanCost >= 8000 || modelMode)
            {
                char timing[192];
                std::snprintf(timing, sizeof(timing),
                              "scan: %llu us (type %llu, find %llu), slots=%u, model=%d",
                              static_cast<unsigned long long>(scanCost),
                              static_cast<unsigned long long>(g_scanTypeCost),
                              static_cast<unsigned long long>(g_scanFindCost),
                              static_cast<unsigned>(g_slots.size()),
                              modelMode ? 1 : 0);
                TraceLine(timing);
            }
            // A fresh slot list invalidates nothing about the positions, but the
            // first reading of a new transform should not wait for the timer.
            // Safe to reset now that the capture runs before Reproject() in the
            // same tick: the extra sample happens immediately, inside this frame.
            // It was NOT safe while the capture was deferred to after Present -
            // the reset then forced a 40-60 ms gap and the skeleton flew ahead.
            g_lastSample = 0;
        }

        // The plates are a SECOND FindObjectsOfType (16-31 ms measured) and they
        // used to ride inside the scan: the scan frame then cost 47 ms and read
        // as a stutter every 3 s. They resolve nothing today (plates: kept 0 of
        // N), so they get their own slow timer and never land in the same frame
        // as a scan.
        // plates from the game's overlay, kept out of the hot path: the scan is a
        // second FindObjectsOfType (16-31 ms measured) and every run so far ends
        // with "plates: kept 0 of 11" - it produces nothing and costs a stutter
        // every 30 s. Flip to true only when a working plate path exists.
        constexpr bool kCollectPlates = true;
        const ULONGLONG platesInterval = 30000;
        if (kCollectPlates && g_options.legacyScan && !modelMode && g_options.names &&
            g_platesTick != 0 &&
            now - g_platesTick >= platesInterval)
        {
            g_platesTick = now;
            const ULONGLONG platesStarted = NowMicroseconds();
            CollectPlates(displayWidth, displayHeight);
            const ULONGLONG platesCost = NowMicroseconds() - platesStarted;
            if (platesCost >= 5000)
            {
                char timing[64];
                std::snprintf(timing, sizeof(timing), "scan: plates cost %llu us",
                              static_cast<unsigned long long>(platesCost));
                TraceLine(timing);
            }
        }
        else if (g_platesTick == 0)
        {
            g_platesTick = now;
        }

        const ULONGLONG sampleIntervalUs =
            static_cast<ULONGLONG>(std::clamp(g_options.sampleIntervalMs, 0, 1000)) * 1000ull;
        // The gate runs on the microsecond clock. GetTickCount64 steps by 15.6 ms,
        // so `now - g_lastSample < 8` was true on EVERY frame: the position read
        // ran at the refresh rate (144 Hz here) instead of the configured 125, and
        // the whole per-character managed pass - plus the joint pass that follows
        // it - landed in every single frame. That is the "it got smooth and the
        // fps dropped" report, measured: the gate never fired.
        const ULONGLONG gateUs = NowMicroseconds();
        if (g_lastSample != 0 && gateUs - g_lastSample < sampleIntervalUs)
        {
            // Between position reads the world positions are still valid, so the
            // only thing that must run every frame is the projection: that is
            // what keeps the boxes glued to the camera while it is turning
            // instead of snapping a sample later. The joints are NOT re-read
            // here: 19 managed calls per body per frame on the render thread cost
            // more than the freshness is worth, and the pose phase in Reproject
            // already covers the gap between captures.
            RefreshLead(g_entities);
            if (!g_workerMode)
            {
                CostScope reprojCost(g_costReprojUs);
                Reproject(displayWidth, displayHeight);
            }
            MarkStage("stage: reproject done");
            PublishEntities();
            return;
        }
        // Variant A of the lag fix: the cheap half of the capture - camera
        // matrices and every body's feet - runs HERE, before Reproject, so the
        // frame about to be drawn carries this frame's positions. Only the
        // expensive joint read stays after Present, at the point where the rig
        // is settled between base animation and weapon IK; moving that read
        // here is what produced the 0.33-0.49 m hand flip-flop the phase shift
        // was introduced for.
        if (!g_workerMode)
        {
            {
                CostScope sampleCost(g_costSampleUs);
                Sample(displayWidth, displayHeight);
            }
            MarkStage("stage: sample done");
            g_capturePending = true;
            g_captureWidth = displayWidth;
            g_captureHeight = displayHeight;
        }
        else
        {
            // The worker owns the capture end-to-end and has no Present to
            // hang it on; leave that path exactly as it was.
            {
                CostScope sampleCost(g_costSampleUs);
                Sample(displayWidth, displayHeight);
            }
            MarkStage("stage: sample done");
            {
                CostScope boneCost(g_costBoneUs);
                BonePass(displayWidth, displayHeight);
            }
            MarkStage("stage: bonepass done");
            SyncBonesFromSlots();
            MarkStage("stage: bones synced");
        }
        // Stamped after the capture, never before it: the gate measures the age
        // of the data, not the age of the frame that asked for it.
        g_lastSample = NowMicroseconds();
        {
            RefreshLead(g_entities);
            if (!g_workerMode)
            {
                CostScope reprojCost(g_costReprojUs);
                Reproject(displayWidth, displayHeight);
            }
        }
        MarkStage("stage: reproject done");
        PublishEntities();
    }

    // The joint capture, at the one point in the frame where the rig is
    // settled: after the game's own Present returned. Tick() arms the flag,
    // this consumes it once. The feet and the camera matrices were already
    // read before Present (Variant A), so the joints read here are paired with
    // an anchor from the same frame - no 1-frame position lag, no second
    // extrapolation. Nothing here projects or publishes: the next Tick() does
    // that with the joints this pass just wrote.
    void TickAfterPresent()
    {
        if (!g_capturePending || g_workerMode)
        {
            return;
        }
        g_capturePending = false;
        const float displayWidth = g_captureWidth;
        const float displayHeight = g_captureHeight;
        if (displayWidth <= 0.0f || displayHeight <= 0.0f)
        {
            return;
        }
        MarkStage("stage: postpresent enter");
        // Bones AFTER the sample, never before: BonePass stamps the pose anchor
        // from the feet that are current when it reads the joints. Running it
        // first paired fresh joints with the previous tick's feet, so every
        // joint carried the distance the body covered in that tick - 10-30 cm
        // at sprint speed, which is the micro-stutter this pass exists to kill.
        {
            CostScope boneCost(g_costBoneUs);
            BonePass(displayWidth, displayHeight);
        }
        MarkStage("stage: bonepass done");
        SyncBonesFromSlots();
        MarkStage("stage: bones synced");
    }

    // Copies the bodies the radar should show. Every body the ESP has resolved
    // counts, including the ones behind the camera: that is the whole point of a
    // radar. The distance is measured from the eye here rather than reusing
    // entity.distance, which is -1 whenever the feet missed the near plane.
    void PublishRadar(const std::vector<Entity> &entities)
    {
        g_radarCount = 0;
        g_radarEyeX = g_cameraPosition.x;
        g_radarEyeZ = g_cameraPosition.z;
        g_radarYaw = g_cameraYawDeg;
        g_radarYawValid = g_cameraYawValid;
        const bool haveEye = g_cameraPositionValid;
        for (const Entity &entity : entities)
        {
            if (g_radarCount >= kMaxRadarContacts)
            {
                break;
            }
            float distance = entity.distance;
            if (haveEye)
            {
                const float dx = entity.feet.x - g_cameraPosition.x;
                const float dz = entity.feet.z - g_cameraPosition.z;
                distance = std::sqrt(dx * dx + dz * dz);
            }
            if (!(distance >= 0.0f) || distance > g_options.maxDistance)
            {
                continue;
            }
            RadarContact &contact = g_radarContacts[g_radarCount++];
            contact.x = entity.feet.x;
            contact.z = entity.feet.z;
            contact.type = static_cast<int>(entity.type);
            contact.distance = distance;
            contact.height = entity.feet.y - g_cameraPosition.y;
        }
        radar::Publish(g_radarContacts, g_radarCount, g_radarEyeX, g_radarEyeZ, g_radarYaw,
                       g_radarYawValid);
    }

    int RadarContacts(RadarContact *out, int capacity, float *eyeX, float *eyeZ, float *yawDegrees)
    {
        if (eyeX != nullptr)
        {
            *eyeX = g_radarEyeX;
        }
        if (eyeZ != nullptr)
        {
            *eyeZ = g_radarEyeZ;
        }
        if (yawDegrees != nullptr)
        {
            *yawDegrees = g_radarYaw;
        }
        if (out == nullptr || capacity <= 0)
        {
            return 0;
        }
        const int take = g_radarCount < capacity ? g_radarCount : capacity;
        for (int index = 0; index < take; ++index)
        {
            out[index] = g_radarContacts[index];
        }
        return take;
    }

    void Draw()
    {
        CostScope drawCost(g_costDrawUs);
        ++g_costFrames;
        MarkStage("stage: draw enter");

        const ImGuiIO &io = ImGui::GetIO();
        const float displayWidth = io.DisplaySize.x;
        const float displayHeight = io.DisplaySize.y;

        // When the worker thread is active, the render thread works on its own
        // copy of the entity list (g_renderEntities), which was filled by
        // FetchEntities() in Tick().  When everything runs on the present
        // thread, g_entities is used directly.
        std::vector<Entity> &drawEntities = g_workerMode ? g_renderEntities : g_entities;

        // The lead belongs to the list that is actually drawn: recomputing it on
        // the worker's g_entities here would write a vector the worker is filling
        // at the same time, and the result would never reach the screen anyway.
        RefreshLead(drawEntities);

        // The head radius reads the same matrix the joints were projected with.
        // Taking g_mvp here while the worker rewrites it in RefreshMatrices()
        // mixed two camera positions in one frame: torn rows showed up as the
        // circle jumping off the skull on a quick turn.
        const float *drawMvp = g_workerMode ? g_renderMvp : g_mvp;
        const bool drawMvpValid = g_workerMode ? g_renderMvpValid : g_mvpValid;

        // Fov ring: a calibration aid, so it is drawn only when the user turns
        // it on (and the aim itself is on, otherwise the radius means nothing).
        ImDrawList *draw = ImGui::GetBackgroundDrawList();
        if (draw == nullptr)
        {
            return;
        }
        // Radar reads the same resolved list the boxes do; publishing before the
        // early return keeps it alive when only the radar is on.
        PublishRadar(drawEntities);
        radar::DrawScreen(displayWidth, displayHeight);

        if (g_options.aimEnabled && g_options.aimFovCircle)
        {
            draw->AddCircle(ImVec2(displayWidth * 0.5f, displayHeight * 0.5f),
                            g_options.aimFovPixels, OptionColor(g_options.colorFov, 90), 64, 1.0f);
        }

        if (!g_options.enabled || drawEntities.empty())
        {
            return;
        }

        int drawn = 0;
        int skinned = 0;
        // One full geometry snapshot per second, so the drawn circle can be
        // compared with the character in a screenshot instead of guessed at.
        static ULONGLONG g_lastHeadDump = 0;
        const ULONGLONG dumpNow = GetTickCount64();
        const bool headDump = (dumpNow - g_lastHeadDump) >= 1000;
        if (headDump)
        {
            g_lastHeadDump = dumpNow;
        }
        int dumpIndex = 0;

        for (Entity &entity : drawEntities)
        {
            if (!entity.visible)
            {
                continue;
            }
            if (entity.distance > 0.0f && entity.distance > g_options.maxDistance)
            {
                continue;
            }
            // Off screen: the projection of an out-of-view character still
            // produces coordinates, and clamping them to the viewport painted a
            // box and a name stuck to the edge of the screen that followed every
            // camera turn. A body that does not touch the frame at all is simply
            // not drawn; it comes back the moment it is on screen again.
            //   The test is the rectangle against the viewport, not its centre
            // point: at arm's length a body fills the screen and its centre
            // leaves the viewport while most of it is still visible, and the
            // centre test dropped exactly the targets you were standing on.
            if (entity.boxRight < 0.0f || entity.boxLeft > displayWidth ||
                entity.boxBottom < 0.0f || entity.boxTop > displayHeight)
            {
                continue;
            }
            ++drawn;
            if (entity.boxFromBones)
            {
                ++skinned;
            }

            // The box quad comes straight from the projection, measured off the
            // joints (see Reproject). Only the viewport clamp happens here, so a
            // character half off the top of the screen still gets a usable
            // rectangle instead of one painted past the border.
            const float top = std::clamp(entity.boxTop, 0.0f, displayHeight);
            const float bottom = std::clamp(entity.boxBottom, top + 1.0f, displayHeight);
            const float left = std::clamp(entity.boxLeft, 0.0f, displayWidth);
            const float right = std::clamp(entity.boxRight, left + 1.0f, displayWidth);
            const float centerX = (left + right) * 0.5f;
            const float height = std::max(bottom - top, 1.0f);

            const ImU32 color = entity.locked
                                    ? OptionColor(g_options.colorLocked, 245)
                                    : OptionColor(TypeColor(entity.type, g_options.colorBox), 230);
            const ImU32 nameColor = OptionColor(g_options.colorName, 240);
            const ImU32 distColor = OptionColor(g_options.colorDistance, 210);
            // One stroke multiplier for every line, box and circle: a hairline
            // vanishes against bright scenery and reads as "the esp flickers".
            const float stroke = std::clamp(g_options.thickness, 0.5f, 4.0f);

            // Skull geometry. The radius is a PHYSICAL size projected at the
            // body's own depth: 0.095 m is the skull half-width measured on the
            // live rig (28 px radius on 310 px/m), so the circle is the same on
            // every skin. It used to be a fraction of the neck->crown screen
            // span, which is a different bone chain per model - the circle
            // covered the whole head on one player and sat on the chin on
            // another, and grew to r=99 px when the head node left the screen.
            // The circle is placed off the NECK, not off the head node: on a
            // seated character (car) that node sits a head height above the
            // skull, and the circle was drawn up over the roof.
            const float bodyPixels = std::max(bottom - top, 8.0f);
            ImVec2 crown(centerX, top);
            bool headValid = false;
            float headRadius = 0.0f;
            if (entity.boneOnScreen[BoneNeck])
            {
                // Radius: the angular size of a 9.5 cm skull at the body's own
                // depth. Row 0 of MVP is the screen-X axis already scaled by the
                // focal length, so |row0| * 0.5 * width / depth is pixels per
                // metre in the plane facing the camera, at any camera tilt.
                //   Both earlier forms were wrong in a measurable way. One metre
                // projected above the feet is not linear: at 1 m from the eye the
                // top of that metre is twice as close as the feet, the scale came
                // out 2x too large (live log: r=76.8 px on an 885 px body).
                // Dividing the BOX height by 1.8 m reads the projection of the
                // body axis, which collapses when the camera looks down it: at
                // 6..10 m from above the whole body is 46 px (box=913,621,925,667)
                // so the circle hit the 2 px floor.
                constexpr float kHeadRadiusMetres = 0.095f;
                if (drawMvpValid && entity.distance > 0.25f)
                {
                    const float row = std::sqrt(drawMvp[0] * drawMvp[0] + drawMvp[4] * drawMvp[4] +
                                                drawMvp[8] * drawMvp[8]);
                    const float perMetre = row * 0.5f * displayWidth / entity.distance;
                    headRadius = std::clamp(perMetre * kHeadRadiusMetres, 2.0f, 64.0f);
                }
                else
                {
                    headRadius = std::clamp(bodyPixels * 0.07f, 2.0f, 24.0f);
                }
                // Upper bound from the measured head span. The analytic radius
                // is right on a whole body, but at arm's length the box is
                // clipped by the screen bottom while entity.distance keeps
                // shrinking, so the circle could grow past the chin. A skull is
                // never wider than ~0.6 of its own neck-to-crown height.
                if (entity.boneOnScreen[BoneNeck] && entity.boneOnScreen[BoneHead] &&
                    entity.boneY[BoneHead] < entity.boneY[BoneNeck])
                {
                    const float spanX = entity.boneX[BoneHead] - entity.boneX[BoneNeck];
                    const float spanY = entity.boneY[BoneHead] - entity.boneY[BoneNeck];
                    const float span = std::sqrt(spanX * spanX + spanY * spanY);
                    if (span > 4.0f)
                    {
                        headRadius = std::min(headRadius, span * 0.62f);
                    }
                }
                // Height from the measured crown, horizontal from the neck.
                // m_head_2_jnt is the crown (rig dump: ExpectedHeight 1.00, it is
                // the topmost point of the box), but it also leans forward with
                // the head: in profile it sat 51 px beside the neck at 3 m, so a
                // circle hung straight off it and the neck link ran diagonally to
                // it - the bend Olivia saw. Half a radius of lateral drift keeps
                // the skull at the crown without letting it leave the neck axis.
                float crownY = entity.boneY[BoneNeck] - headRadius * 2.0f;
                float centreX = entity.boneX[BoneNeck];
                if (entity.boneOnScreen[BoneHead] &&
                    entity.boneY[BoneHead] < entity.boneY[BoneNeck])
                {
                    crownY = entity.boneY[BoneHead];
                    const float shift = entity.boneX[BoneHead] - entity.boneX[BoneNeck];
                    centreX = entity.boneX[BoneNeck] +
                              std::clamp(shift, -headRadius * 0.5f, headRadius * 0.5f);
                }
                crown = ImVec2(centreX, crownY);
                headValid = true;
            }
            if (!headValid && entity.boneOnScreen[BoneNeck])
            {
                float up = bodyPixels * 0.10f;
                if (entity.boneOnScreen[BoneHips])
                {
                    const float span = std::fabs(entity.boneY[BoneHips] - entity.boneY[BoneNeck]);
                    if (span > 1.0f)
                    {
                        up = std::clamp(span * 0.34f, bodyPixels * 0.06f, bodyPixels * 0.16f);
                    }
                }
                crown = ImVec2(entity.boneX[BoneNeck], entity.boneY[BoneNeck] - up);
                crown.y = std::clamp(crown.y, top - bodyPixels * 0.05f, bottom);
                headRadius = std::clamp(up * 0.55f, 2.0f, std::max(bodyPixels * 0.12f, 2.0f));
                headValid = true;
            }
            else if (!headValid && entity.boneOnScreen[BoneHead])
            {
                // No neck to measure against: the head node is the only anchor
                // left, kept inside the box so it cannot float above the body.
                crown = ImVec2(entity.boneX[BoneHead],
                               std::clamp(entity.boneY[BoneHead], top, bottom));
                headRadius = std::clamp(bodyPixels * 0.05f, 2.0f, std::max(bodyPixels * 0.12f, 2.0f));
                headValid = true;
            }
            const ImVec2 headCentre(crown.x, crown.y + headRadius);
            const float topEdge =
                headValid ? std::clamp(crown.y, 0.0f, displayHeight)
                          : std::clamp(top, 0.0f, displayHeight);

            if (headDump)
            {
                // The count of MEASURED joints, not "is the head present": the
                // old field printed 1 whenever the crown projected, which read
                // as "one joint on screen" while the figure was actually whole,
                // and hid the gate that was dropping the whole skeleton.
                int measured = 0;
                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    if (entity.boneOnScreen[bone])
                    {
                        ++measured;
                    }
                }
                char line[320];
                std::snprintf(
                    line, sizeof(line),
                    "head: #%d d=%.0f crown=%.0f,%.0f neck=%.0f,%.0f shl=%.0f,%.0f shr=%.0f,%.0f "
                    "c=%.0f,%.0f r=%.1f top=%.0f box=%.0f,%.0f,%.0f,%.0f joints=%d skull=%d",
                    dumpIndex, entity.distance, crown.x, crown.y,
                    entity.boneOnScreen[BoneNeck] ? entity.boneX[BoneNeck] : -1.0f,
                    entity.boneOnScreen[BoneNeck] ? entity.boneY[BoneNeck] : -1.0f,
                    entity.boneOnScreen[BoneLeftUpperArm] ? entity.boneX[BoneLeftUpperArm] : -1.0f,
                    entity.boneOnScreen[BoneLeftUpperArm] ? entity.boneY[BoneLeftUpperArm] : -1.0f,
                    entity.boneOnScreen[BoneRightUpperArm] ? entity.boneX[BoneRightUpperArm] : -1.0f,
                    entity.boneOnScreen[BoneRightUpperArm] ? entity.boneY[BoneRightUpperArm] : -1.0f,
                    headCentre.x, headCentre.y, headRadius, topEdge, left, topEdge, right, bottom,
                    measured, headValid ? 1 : 0);
                TraceLine(line);
                if (dumpIndex == 0)
                {
                    // The drawn chain in the order it is stacked on screen. A
                    // correct composition reads hips > spine > chest > neck >
                    // crown in Y (screen Y grows downward), so this line is the
                    // check itself, not a proxy for it.
                    char chain[160];
                    std::snprintf(chain, sizeof(chain),
                                  "chain: d=%.0f hips=%.0f spine=%.0f chest=%.0f neck=%.0f "
                                  "crown=%.0f feet=%.0f",
                                  entity.distance,
                                  entity.boneOnScreen[BoneHips] ? entity.boneY[BoneHips] : -1.0f,
                                  entity.boneOnScreen[BoneSpine] ? entity.boneY[BoneSpine] : -1.0f,
                                  entity.boneOnScreen[BoneChest] ? entity.boneY[BoneChest] : -1.0f,
                                  entity.boneOnScreen[BoneNeck] ? entity.boneY[BoneNeck] : -1.0f,
                                  entity.boneOnScreen[BoneHead] ? entity.boneY[BoneHead] : -1.0f,
                                  bottom);
                    TraceLine(chain);
                }
                ++dumpIndex;
            }

            if (g_options.boxes)
            {
                draw->AddRect(ImVec2(left, topEdge), ImVec2(right, bottom), color, 0.0f, 0,
                              1.5f * stroke);
            }
            if (g_options.snaplines)
            {
                const ImVec2 origin(displayWidth * 0.5f, displayHeight);
                draw->AddLine(origin, ImVec2(centerX, bottom), color, 1.0f * stroke);
            }
            // The skull ring belongs to the figure, and the marker must not
            // draw a second one over it: two circles of different segment
            // counts read as one thick double ring.
            bool skullDrawn = false;
            if (g_options.skeleton)
            {
                // A figure is only drawn where the rig actually gave joints. When
                // the pose did not resolve (a character seated inside a car, a
                // pooled or respawning rig) the old code invented limbs from the
                // box and drew that web across whatever the box covered — the
                // car. Nothing measured, nothing drawn.
                int jointsOnScreen = 0;
                for (int bone = 0; bone < kBoneCount; ++bone)
                {
                    if (entity.boneOnScreen[bone])
                    {
                        ++jointsOnScreen;
                    }
                }
                // Hysteresis, because the joint read is budgeted and a far
                // character loses a joint now and then: entering at four and
                // leaving at two stops the figure blinking off at range.
                //
                // Four, not six: the hitbox buffer publishes SEVEN bones per
                // character — pelvis, two spine segments, neck, two head
                // segments (63 floats / 9) — and there are no limb bones in it
                // at all. The old gate of six could therefore never open, which
                // is exactly "boxes draw, skeleton never does": five torso
                // joints were measured every frame and thrown away.
                if (jointsOnScreen >= 4)
                {
                    entity.bonesShown = true;
                }
                else if (jointsOnScreen <= 2)
                {
                    entity.bonesShown = false;
                }
                if (!entity.bonesShown)
                {
                    // Not enough measured joints to draw a body; the rest of the
                    // overlay (box, name, distance) still runs below.
                }
                else
                {
                // The reference figure: head circle, ONE straight spine line
                // from the head to the hips, ONE shoulder line, arms hanging
                // off its ends (shoulder -> elbow -> hand), a fork at the hips
                // and two straight legs (hip -> knee -> foot). The old table
                // drew chest -> shoulder diagonals, which read as a web; a
                // joint the rig does not expose is interpolated or placed off
                // the box instead of a diagonal being invented for it.
                const ImU32 boneColor = OptionColor(g_options.colorSkeleton, 235);
                const float boxWidth = std::max(entity.boxRight - entity.boxLeft, 2.0f);
                const auto point = [&](int bone, float fallbackX, float fallbackY) {
                    return entity.boneOnScreen[bone]
                               ? ImVec2(entity.boneX[bone], entity.boneY[bone])
                               : ImVec2(fallbackX, fallbackY);
                };
                // Measured joint, else the midpoint between its parent and the
                // next joint down the chain, else the box-relative fallback.
                const auto joint = [&](int bone, ImVec2 parent, int next, ImVec2 fallback) {
                    if (entity.boneOnScreen[bone])
                    {
                        return ImVec2(entity.boneX[bone], entity.boneY[bone]);
                    }
                    if (next >= 0 && entity.boneOnScreen[next])
                    {
                        return ImVec2((parent.x + entity.boneX[next]) * 0.5f,
                                      (parent.y + entity.boneY[next]) * 0.5f);
                    }
                    return fallback;
                };
                const auto line = [&](ImVec2 a, ImVec2 b) {
                    draw->AddLine(a, b, boneColor, 1.4f * stroke);
                };

                const ImVec2 hips = point(BoneHips, centerX, top + height * 0.55f);
                const ImVec2 shoulderLeft =
                    point(BoneLeftUpperArm, hips.x - boxWidth * 0.21f, top + height * 0.22f);
                const ImVec2 shoulderRight =
                    point(BoneRightUpperArm, hips.x + boxWidth * 0.21f, top + height * 0.22f);

                const float elbowDrop = height * 0.17f;
                const float handDrop = height * 0.15f;
                const ImVec2 elbowLeft =
                    joint(BoneLeftLowerArm, shoulderLeft, BoneLeftHand,
                          ImVec2(shoulderLeft.x, shoulderLeft.y + elbowDrop));
                const ImVec2 elbowRight =
                    joint(BoneRightLowerArm, shoulderRight, BoneRightHand,
                          ImVec2(shoulderRight.x, shoulderRight.y + elbowDrop));
                const ImVec2 handLeft =
                    joint(BoneLeftHand, elbowLeft, -1, ImVec2(elbowLeft.x, elbowLeft.y + handDrop));
                const ImVec2 handRight = joint(BoneRightHand, elbowRight, -1,
                                               ImVec2(elbowRight.x, elbowRight.y + handDrop));

                const ImVec2 hipLeft =
                    point(BoneLeftUpperLeg, hips.x - boxWidth * 0.12f, hips.y);
                const ImVec2 hipRight =
                    point(BoneRightUpperLeg, hips.x + boxWidth * 0.12f, hips.y);
                const float kneeY = (hips.y + bottom) * 0.5f;
                const ImVec2 kneeLeft =
                    joint(BoneLeftLowerLeg, hipLeft, BoneLeftFoot, ImVec2(hipLeft.x, kneeY));
                const ImVec2 kneeRight =
                    joint(BoneRightLowerLeg, hipRight, BoneRightFoot, ImVec2(hipRight.x, kneeY));
                const ImVec2 footLeft =
                    joint(BoneLeftFoot, kneeLeft, -1, ImVec2(kneeLeft.x, bottom));
                const ImVec2 footRight =
                    joint(BoneRightFoot, kneeRight, -1, ImVec2(kneeRight.x, bottom));

                // The skull circle and the spine that ends under it come from
                // the measured crown/neck pair computed above the figure.
                if (headValid)
                {
                    draw->AddCircle(headCentre, headRadius, boneColor, 24, 1.4f * stroke);
                    skullDrawn = true;
                }
                // Only what was MEASURED is drawn. The hitbox buffer publishes a
                // torso chain — pelvis, two spine segments, neck, crown — and no
                // limb bones whatsoever, so rebuilding arms and legs from the
                // box is precisely what drew a web across a car. Where the limbs
                // exist (the view-slot rig with its 19 *_jnt nodes) they are
                // measured and appear on their own.
                // The chain stops at the neck. Neck -> head is drawn as one
                // straight link to the RING EDGE instead of a separate segment
                // through the skull: the old code ended the chain at chest->neck
                // (k + 2 < 5 dropped the last pair) and then linked the neck to
                // the ring's bottom point, which hung off the crown, so the line
                // bent sideways at the neck whenever the head leaned.
                const auto on = [&](int bone) { return entity.boneOnScreen[bone]; };
                const int torso[5] = { BoneHips, BoneSpine, BoneChest, BoneNeck, BoneHead };
                for (int k = 0; k + 2 < 5; ++k)
                {
                    if (on(torso[k]) && on(torso[k + 1]))
                    {
                        line(point(torso[k], 0.0f, 0.0f), point(torso[k + 1], 0.0f, 0.0f));
                    }
                }
                if (headValid)
                {
                    ImVec2 anchor(-1.0f, -1.0f);
                    if (on(BoneNeck))
                    {
                        anchor = point(BoneNeck, 0.0f, 0.0f);
                    }
                    else if (on(BoneChest))
                    {
                        anchor = point(BoneChest, 0.0f, 0.0f);
                    }
                    else if (on(BoneHips))
                    {
                        anchor = hips;
                    }
                    if (anchor.x >= 0.0f)
                    {
                        // Stop on the ring itself, along the line to its centre:
                        // the segment continues the neck bone direction, so there
                        // is no kink and no line through the skull.
                        const float dx = headCentre.x - anchor.x;
                        const float dy = headCentre.y - anchor.y;
                        const float len = std::sqrt(dx * dx + dy * dy);
                        if (len > 1.0f)
                        {
                            const float stop =
                                std::clamp((len - headRadius) / len, 0.0f, 1.0f);
                            line(anchor, ImVec2(anchor.x + dx * stop, anchor.y + dy * stop));
                        }
                    }
                }
                if (on(BoneLeftUpperArm) && on(BoneRightUpperArm))
                {
                    line(shoulderLeft, shoulderRight);
                }
                // Баг 1.4: не рисуем линию shoulder→elbow если elbow
                // отбракован — иначе elbowLeft синтетически вычислен и
                // при мигании локтя рука дёргается к искусственной точке.
                if (on(BoneLeftUpperArm) && on(BoneLeftLowerArm))
                {
                    line(shoulderLeft, elbowLeft);
                }
                if (on(BoneLeftLowerArm) && on(BoneLeftHand))
                {
                    line(elbowLeft, handLeft);
                }
                // Баг 1.4: аналогично для правой руки.
                if (on(BoneRightUpperArm) && on(BoneRightLowerArm))
                {
                    line(shoulderRight, elbowRight);
                }
                if (on(BoneRightLowerArm) && on(BoneRightHand))
                {
                    line(elbowRight, handRight);
                }
                // Баг 1.4: ноги — добавлен guard on(BoneHips) чтобы
                // при отсутствии BoneHips линия не тянулась к центру бокса.
                if (on(BoneLeftUpperLeg) && on(BoneHips))
                {
                    line(hips, hipLeft);
                }
                if (on(BoneRightUpperLeg) && on(BoneHips))
                {
                    line(hips, hipRight);
                }
                if (on(BoneLeftLowerLeg) && on(BoneLeftFoot))
                {
                    line(kneeLeft, footLeft);
                }
                if (on(BoneRightLowerLeg) && on(BoneRightFoot))
                {
                    line(kneeRight, footRight);
                }
                if (on(BoneLeftLowerLeg) && on(BoneLeftFoot) && on(BoneLeftUpperLeg))
                {
                    line(hipLeft, kneeLeft);
                }
                if (on(BoneRightLowerLeg) && on(BoneRightFoot) && on(BoneRightUpperLeg))
                {
                    line(hipRight, kneeRight);
                }
                }
            }
            if (g_options.headMarker && headValid && !skullDrawn)
            {
                // Only when the figure is off. With the skeleton on, its own
                // ring IS the marker, and a second circle at the same centre
                // and radius drew the double ring on Olivia's screenshot.
                draw->AddCircle(headCentre, headRadius, color, 24, 1.4f * stroke);
            }
            // Labels are centred over the box and stacked upward, one above the
            // other. They used to be pinned to the box's LEFT edge, so with a
            // narrow box at long range the text hung off to one side of the
            // character and read as "not level".
            const auto centred = [&](float y, const char *text, ImU32 ink = 0) -> float {
                const ImVec2 size = ImGui::CalcTextSize(text);
                const float x = std::clamp(centerX - size.x * 0.5f, 2.0f,
                                           std::max(displayWidth - size.x - 2.0f, 2.0f));
                draw->AddText(ImVec2(x, y), ink != 0 ? ink : nameColor, text);
                return size.y;
            };

            float labelY = std::max(topEdge - 15.0f, 2.0f);
            const char *topLabel = entity.nickname[0] != '\0'
                                       ? entity.nickname
                                       : (entity.name[0] != '\0' ? entity.name : nullptr);
            if (g_options.names && topLabel != nullptr)
            {
                labelY -= centred(labelY, topLabel);
            }
            if (g_options.roleTags && entity.type != RoleType::Unknown)
            {
                // The role word the game itself uses, in that side's colour, so
                // the box and the tag never disagree.
                labelY -= centred(labelY, RoleLabel(entity.type), color) + 1.0f;
            }
            else if (entity.role[0] != '\0')
            {
                labelY -= centred(labelY, entity.role) + 1.0f;
            }
            // Armour is a flag in this build (HasArmor 0/1), so "armor 0" over
            // every bare body was pure noise. Only the armoured ones get a tag.
            if (entity.armor > 0)
            {
                labelY -= centred(labelY, "ARMOR", distColor) + 1.0f;
            }
            if (g_options.status && entity.speed > 0.35f)
            {
                char text[32];
                std::snprintf(text, sizeof(text), "MOVING %.1f m/s", entity.speed);
                centred(std::min(bottom + 2.0f, displayHeight - 14.0f), text, distColor);
            }
            if (g_options.distance)
            {
                char text[32];
                std::snprintf(text, sizeof(text), "%.0f m", entity.distance);
                const float y = g_options.status && entity.speed > 0.35f
                                    ? std::min(bottom + 16.0f, displayHeight - 14.0f)
                                    : std::min(bottom + 2.0f, displayHeight - 14.0f);
                centred(y, text, distColor);
            }

            // Locked entity gets the diagnostic marker the ticket asks for: the
            // lead point itself, drawn so the fov and prediction settings can be
            // judged without firing a shot.
            if (entity.locked && g_options.aimDrawTarget && g_options.aimEnabled)
            {
                float boneX = 0.0f;
                float boneY = 0.0f;
                aim::BonePoint(entity.predictedHeadX, entity.predictedHeadY, entity.feetX,
                               entity.feetY, g_options.aimOnHead ? g_options.aimBone : 2, boneX,
                               boneY);
                draw->AddCircle(ImVec2(boneX, boneY), 6.0f, IM_COL32(255, 120, 120, 245), 16,
                                1.5f);
                draw->AddCircleFilled(ImVec2(boneX, boneY), 2.0f, IM_COL32(255, 120, 120, 245));
            }
        }

        // One line a second with how many entities actually reached the draw
        // list: separates "walk found nobody" from "nobody was on screen".
        static ULONGLONG lastDrawTrace = 0;
        const ULONGLONG nowMs = GetTickCount64();
        if (nowMs - lastDrawTrace >= 1000)
        {
            lastDrawTrace = nowMs;
            char text[64];
            std::snprintf(text, sizeof(text), "draw: %d of %d, skinned=%d", drawn,
                          static_cast<int>(drawEntities.size()), skinned);
            Trace(text);
        }

        // Split of the frame, five-second window, microseconds per frame. This is
        // the line that replaces "somewhere in the esp": it says whether the next
        // fix belongs to the managed walk, the joint reads, the projection or the
        // draw list.
        static ULONGLONG partsWindow = 0;
        const ULONGLONG partsNow = GetTickCount64();
        if (partsWindow == 0)
        {
            partsWindow = partsNow;
        }
        if (partsNow - partsWindow >= 5000 && g_costFrames > 0)
        {
            const unsigned frames = g_costFrames;
            char text[192];
            std::snprintf(text, sizeof(text),
                          "esp: parts sample=%llu bone=%llu reproj=%llu draw=%llu us/frame "
                          "over %u frames",
                          static_cast<unsigned long long>(g_costSampleUs / frames),
                          static_cast<unsigned long long>(g_costBoneUs / frames),
                          static_cast<unsigned long long>(g_costReprojUs / frames),
                          static_cast<unsigned long long>(g_costDrawUs / frames), frames);
            Trace(text);
            partsWindow = partsNow;
            g_costSampleUs = 0;
            g_costBoneUs = 0;
            g_costReprojUs = 0;
            // The draw cost includes the frame that reports it; keeping it would
            // bias the next window by one frame, so it is cleared with the rest.
            g_costDrawUs = 0;
            g_costFrames = 0;
        }
    }

    // Screen point the crosshair is pulled to for one body. The measured joint
    // wins when the pose has it on screen; the head-to-feet line is the
    // fallback, and it is the only source for the shared-table poses that carry
    // no limb joints at all. `bone` 3 asks for whichever of head / chest / hips
    // is closest to the centre this frame.
    static void AimPoint(const Entity &entity, int bone, float centreX, float centreY, float &outX,
                         float &outY)
    {
        if (bone == 3)
        {
            float bestX = 0.0f;
            float bestY = 0.0f;
            float best = 1.0e30f;
            for (int option = 0; option <= 2; ++option)
            {
                float x = 0.0f;
                float y = 0.0f;
                AimPoint(entity, option, centreX, centreY, x, y);
                const float dx = x - centreX;
                const float dy = y - centreY;
                const float square = dx * dx + dy * dy;
                if (square < best)
                {
                    best = square;
                    bestX = x;
                    bestY = y;
                }
            }
            outX = bestX;
            outY = bestY;
            return;
        }

        const int joint = bone == 0 ? BoneHead : (bone == 1 ? BoneChest : BoneHips);
        if (g_options.aimUseBones && entity.boneOnScreen[joint] && entity.boneVisible[joint])
        {
            // The joints are drawn without the lead, so the head's pixel lead is
            // added back here. Same velocity, same frame: the offset is valid for
            // every joint of one rigid body.
            outX = entity.boneX[joint];
            outY = entity.boneY[joint];
            if (g_options.aimPrediction)
            {
                outX += entity.leadX;
                outY += entity.leadY;
            }
            return;
        }

        aim::BonePoint(entity.predictedHeadX, entity.predictedHeadY, entity.feetX, entity.feetY, bone,
                       outX, outY);
    }

    void Aim(float displayWidth, float displayHeight, float deltaSeconds)
    {
        std::vector<Entity> &aimEntities = g_workerMode ? g_renderEntities : g_entities;        if (!g_options.aimEnabled || aimEntities.empty() || displayWidth <= 1.0f ||
            displayHeight <= 1.0f)
        {
            return;
        }
        if ((GetAsyncKeyState(g_options.aimKey) & 0x8000) == 0)
        {
            for (Entity &entity : aimEntities)
            {
                entity.locked = false;
            }
            return;
        }

        const float centerX = displayWidth * 0.5f;
        const float centerY = displayHeight * 0.5f;
        const float fovRadius = std::max(g_options.aimFovPixels, 1.0f);
        const int bone = g_options.aimOnHead ? g_options.aimBone : 2;

        Entity *target = nullptr;
        float best = 0.0f;
        float nearest = 1.0e30f;

        for (Entity &entity : aimEntities)
        {
            entity.locked = false;
            if (!entity.visible)
            {
                continue;
            }
            if (entity.distance > 0.0f && entity.distance > g_options.aimMaxDistance)
            {
                continue;
            }
            // Health is read on the model path only; -1 means "not read", which
            // must not be mistaken for dead.
            if (g_options.aimSkipDead && entity.health >= 0 && entity.health <= 0)
            {
                continue;
            }

            float aimX = 0.0f;
            float aimY = 0.0f;
            AimPoint(entity, bone, centerX, centerY, aimX, aimY);
            const float dx = aimX - centerX;
            const float dy = aimY - centerY;
            const float square = dx * dx + dy * dy;

            if (square < nearest)
            {
                nearest = square;
            }
            if (square > fovRadius * fovRadius)
            {
                continue;
            }

            // Nearest-to-crosshair wins; distance only settles a tie between
            // two characters sitting at the same screen offset.
            const float score = aim::Score(dx, dy, entity.distance, 0.25f);
            if (target == nullptr || score < best)
            {
                best = score;
                target = &entity;
            }
        }

        // Sticky lock. The vector is rebuilt on every sample, so the lock is
        // held by component pointer, not by Entity*. A candidate has to beat the
        // current body by the switch margin before it takes the pull: a crouch
        // re-ranks the list for one frame, and without the margin the crosshair
        // jumps to whoever is closest for that frame.
        static void *aimLockKey = nullptr;
        if (target != nullptr && g_options.aimSwitchMargin > 0.0f)
        {
            Entity *locked = nullptr;
            for (Entity &entity : aimEntities)
            {
                if (entity.key == aimLockKey && entity.visible)
                {
                    locked = &entity;
                    break;
                }
            }
            if (locked != nullptr && locked != target)
            {
                float lockedX = 0.0f;
                float lockedY = 0.0f;
                AimPoint(*locked, bone, centerX, centerY, lockedX, lockedY);
                const float lockedDistance = std::sqrt((lockedX - centerX) * (lockedX - centerX) +
                                                       (lockedY - centerY) * (lockedY - centerY));
                const float candidateDistance = std::sqrt(nearest);
                const bool lockedInRange = lockedDistance <= fovRadius;
                if (lockedInRange && candidateDistance + g_options.aimSwitchMargin >= lockedDistance)
                {
                    // The previous body still owns the pull: clear the flag on
                    // the frame's winner and resign it to the lock.
                    target->locked = false;
                    target = locked;
                    for (Entity &entity : aimEntities)
                    {
                        entity.locked = false;
                    }
                    nearest = lockedDistance * lockedDistance;
                }
            }
        }
        if (target != nullptr)
        {
            aimLockKey = target->key;
        }
        else
        {
            aimLockKey = nullptr;
        }

        // One line a second while the key is held: "no lock" from "nothing in
        // range" are different failures and only the distance tells them apart.
        static ULONGLONG lastAimTrace = 0;
        const ULONGLONG nowMs = GetTickCount64();
        if (nowMs - lastAimTrace >= 1000)
        {
            lastAimTrace = nowMs;
            char text[96];
            std::snprintf(text, sizeof(text), "aim: nearest=%.0fpx fov=%.0fpx lock=%s",
                          nearest < 1.0e29f ? std::sqrt(nearest) : -1.0f,
                          g_options.aimFovPixels, target != nullptr ? "yes" : "no");
            Trace(text);
        }

        if (target == nullptr)
        {
            std::snprintf(g_aimDebug, sizeof(g_aimDebug), "aim: no target in fov");
            return;
        }

        target->locked = true;

        float aimX = 0.0f;
        float aimY = 0.0f;
        AimPoint(*target, bone, centerX, centerY, aimX, aimY);

        // Humanisation: a fresh offset inside the hitbox every period. The LCG
        // is deliberate: rand() is shared with the CRT and would be reseeded by
        // anything else in the process, and the value only has to look unsteady.
        static ULONGLONG jitterTick = 0;
        static float jitterX = 0.0f;
        static float jitterY = 0.0f;
        if (g_options.aimJitter)
        {
            const ULONGLONG now = GetTickCount64();
            const int period = std::max(g_options.aimJitterPeriodMs, 16);
            if (jitterTick == 0 || now - jitterTick >= static_cast<ULONGLONG>(period))
            {
                jitterTick = now == 0 ? 1 : now;
                static unsigned int seed = 0x9E3779B9u;
                seed = seed * 1664525u + 1013904223u;
                const float first = static_cast<float>((seed >> 16) & 0x7FFF) / 16383.5f - 1.0f;
                seed = seed * 1664525u + 1013904223u;
                const float second = static_cast<float>((seed >> 16) & 0x7FFF) / 16383.5f - 1.0f;
                jitterX = first * g_options.aimJitterPixels;
                jitterY = second * g_options.aimJitterPixels;
            }
            aimX += jitterX;
            aimY += jitterY;
        }

        float dx = aimX - centerX;
        float dy = aimY - centerY;

        // Snap mode still honours the angular budget: it removes the smoothing
        // divisor, not the per-frame clamp, because a single 200 px event is
        // discarded by the game's own input handling.
        if (g_options.aimMode == 0)
        {
            aim::StepPixels(dx, dy, g_options.aimSmoothing, g_options.aimMaxStep);
        }
        else
        {
            aim::StepPixels(dx, dy, 1.0f, g_options.aimMaxStep);
        }

        const float frameSeconds = std::clamp(deltaSeconds, 0.001f, 0.100f);
        aim::ClampAngular(dx, dy, g_options.aimPixelsPerDegree,
                          std::max(g_options.aimMaxDegreesPerSecond, 0.0f) * frameSeconds);

        std::snprintf(g_aimDebug, sizeof(g_aimDebug),
                      "aim: id=%d lead=%.1f,%.1fpx step=%.1f,%.1fpx speed=%.1f", target->id,
                      target->leadX, target->leadY, dx, dy, target->speed);

        if (std::fabs(dx) < 0.5f && std::fabs(dy) < 0.5f)
        {
            return;
        }

        INPUT input{};
        input.type = INPUT_MOUSE;
        input.mi.dx = static_cast<LONG>(dx);
        input.mi.dy = static_cast<LONG>(dy);
        input.mi.dwFlags = MOUSEEVENTF_MOVE;
        SendInput(1, &input, sizeof(INPUT));
    }

    const char *Status() { return g_status; }
    int EntityCount()
    {
        return static_cast<int>(g_workerMode ? g_renderEntities.size() : g_entities.size());
    }
    const char *DebugLine() { return g_debug; }
    const char *AimDebugLine() { return g_aimDebug; }
}

