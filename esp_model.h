// Read-only view of the battle model graph.
//
// The ESP used to be built on the rendered character views: one
// FindObjectsOfType, one Transform per view, one Transform.get_position per
// sample. That path cannot answer "which of these is me" and it loses a box
// whenever a managed call faults, which is what the flicker and the box on the
// own body were.
//
// The game already keeps the authoritative list: BattleModel -> BattleEntitiesModel
// -> WorldEntitiesModel -> Dictionary<int, BattleCharacterModel>, plus the exact
// local player at BattleEntitiesModel.<Player> -> PlayerEntityModel.<ServerModel>.
// Every pointer there is a plain managed reference, so after one successful
// locate the whole snapshot is ReadProcessMemory on known addresses: no managed
// calls at all, no allocation, nothing that can fault the render thread.
//
// Field offsets are resolved by NAME through the il2cpp exports, so a game
// update that keeps the class and field names keeps this working.
// Only the BCL Dictionary/Array layout is a constant, because it ships with the
// runtime rather than with the game (and it matches the proven external reader).
#pragma once

#include "il2cpp_api.h"

namespace omega::esp::model
{
    struct Character
    {
        void *character = nullptr;  // BattleCharacterModel*
        int id = 0;                 // dictionary key (entity id)
        Vec3 feet{};
        char role[24] = {};         // PlayerRoleDescription.<Id>, empty = not read
        char nickname[40] = {};     // BattleCharacterSlotData.<Nickname>, empty = not read
        int armor = -1;             // 1 = HasArmor true, 0 = false, -1 = not read
        int health = -1;            // -1 = not read
        // Role and armor come from several pointer hops per character; they do
        // not change mid-match, so they are re-read on a slow timer instead of
        // on every 33 ms snapshot (12 characters * ~8 reads was the 5.6 ms peak).
        unsigned long long profileTick = 0;
        // Last time the transform actually answered. A failed read keeps the
        // previous feet, but only while they are this fresh: a character whose
        // model is gone (left the lobby, respawned into a new object, got into a
        // car) used to keep a box standing at its last position forever, which
        // is the "the box stayed behind while the body drove off" report.
        unsigned long long feetTick = 0;
    };

    void Attach(Il2CppResolver *resolver, void (*log)(const char *message));
    // Drops the cached instance; the next Tick locates the graph again.
    void Reset();

    // Advances the graph: at most kScanBytesPerTick of a fresh locate scan per
    // call, and a snapshot refresh when its TTL expired. Cheap and bounded.
    void Tick(unsigned long long nowMs);

    bool Ready();
    // The model graph was requested by the flag file and is not disabled by a
    // sentinel. When this is true the caller must not run the managed walk at
    // all: it is the only path that faults and deadlocks.
    bool Requested();
    size_t Count();
    const Character *At(size_t index);
    // Snapshot generation: changes every time the list is refreshed, so the
    // caller can tell a stale cache from a fresh one.
    unsigned long long Version();

    // Exact local-player test. Null and unknown pointers are never local.
    bool IsLocal(const void *character);

    // The local BattleCharacterModel, or null while the graph is not located.
    // The skeleton pose module needs it: the local player is the only character
    // whose SkeletonModel the game fills, so the shared rig is read from there.
    void *Local();

    // One line for the diagnostics tab: state, characters, last failure.
    const char *Status();
}
