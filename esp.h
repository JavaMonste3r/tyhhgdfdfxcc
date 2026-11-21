// ESP and aim, driven entirely through the il2cpp exports of GameAssembly.dll.
//
// Nothing here hardcodes a game offset: entities are found by managed class name
// and positioned by asking the running runtime for their Transform, so a game
// update that keeps the class name keeps this working. The class names come from
// DummyDll/dump.cs and are editable from the overlay's esp tab.
#pragma once

#include "il2cpp_api.h"

namespace omega::esp
{
    // UnityEngine.HumanBodyBones 0..18 (Hips .. RightHand) are consecutive, so
    // the bone id is the array index. UpperChest (54) and the toe joints are
    // not drawn: they add nothing a player silhouette needs.
    constexpr int kBoneCount = 19;
    enum Bone : int
    {
        BoneHips = 0,
        BoneLeftUpperLeg = 1,
        BoneRightUpperLeg = 2,
        BoneLeftLowerLeg = 3,
        BoneRightLowerLeg = 4,
        BoneLeftFoot = 5,
        BoneRightFoot = 6,
        BoneSpine = 7,
        BoneChest = 8,
        BoneNeck = 9,
        BoneHead = 10,
        BoneLeftShoulder = 11,
        BoneRightShoulder = 12,
        BoneLeftUpperArm = 13,
        BoneRightUpperArm = 14,
        BoneLeftLowerArm = 15,
        BoneRightLowerArm = 16,
        BoneLeftHand = 17,
        BoneRightHand = 18,
    };

    // PlayerRoleDescription.<Id> folded into the three sides the tabs draw.
    // The role string itself still reaches the label; this only picks a colour.
    enum class RoleType : int
    {
        Unknown = 0,
        Innocent,
        Criminal,
        Cop,
    };

    struct Options
    {
        // --- diagnostics ---------------------------------------------------
        bool enabled = false;
        bool boxes = true;
        bool names = true;
        bool status = true;
        bool distance = true;
        bool snaplines = false;
        // Skeleton: the character's biped joints from Animator.GetBoneTransform.
        // Costs one position read per joint per sample, so it only runs while
        // this is on.
        bool skeleton = true;
        bool headMarker = true;
        // v2 bone source: the skeleton each character carries itself
        // (BattleCharacterModel.<SkeletonModel>._bonesData) instead of the
        // shared hitbox table. Same bone records, but owned by one body, so no
        // row assignment and no neighbour's pose. Off keeps the current table
        // path byte-for-byte unchanged.
        bool espV2 = false;
        // Line/box stroke multiplier. 1.0 is a hairline that disappears against
        // bright scenery; the request was "draw it a bit thicker".
        float thickness = 1.7f;
        float maxDistance = 1000.0f;
        // Drops the character the third-person camera follows. On by default:
        // the own body is already visible on screen, so a box and a skeleton
        // over it only clutter the view. The checkbox turns it back on when
        // the rig needs checking against the model.
        bool selfFilter = true;
        // Last-resort own-body gate: the closest character inside this many
        // metres of the eye. Only used when neither the Animator nor the
        // Transform chain identified the local player. The third-person camera
        // sits up to ~6 m behind the body when zoomed out, so this has to stay
        // clear of that distance or the own body slips through the gate.
        float selfDistance = 8.0f;
        // Managed registry walk (FindObjectsOfType) cadence. It allocates a
        // managed array and races the game thread, so it runs rarely: between
        // scans the cached component list is sampled with cheap position reads.
        // 3 s keeps a newly spawned player inside one respawn cycle; the walk is
        // guarded by the fault barrier, which is what made 8 s the safe number.
        int scanIntervalMs = 5000;
        // FindObjectsOfType is the one call that took the process down: it is a
        // managed array allocation executed on the render thread, which is a
        // foreign thread for il2cpp. pid 24408 hung inside it ("scan: find", no
        // further lines), pid 18312 was killed right after "fault: at scan: find".
        // It is safe again now that the whole walk runs on a system pool thread
        // (esp::StartWorker) instead of Present.
        bool legacyScan = true;
        // Position/projection cadence. Pure reads plus native math, no
        // allocation, so this can be fast enough to feel live. Projection itself
        // runs every frame; this only bounds how often world positions refresh.
        int sampleIntervalMs = 8;   // position reads: 125 Hz, the staircase the
                                    // bones and the box both ride on

        // --- aim -----------------------------------------------------------
        bool aimEnabled = false;
        int aimKey = 0x05;          // VK_XBUTTON2 (mouse button 5)
        int aimMode = 0;            // 0 = smooth, 1 = snap
        float aimFovPixels = 160.0f;
        float aimSmoothing = 6.0f;
        float aimMaxStep = 40.0f;   // pixels per frame, before the angle budget
        float aimMaxDistance = 400.0f;
        bool aimOnHead = true;      // kept so older config files still load
        // 0 head, 1 chest, 2 feet, 3 whichever of head / chest / hips sits
        // closest to the crosshair this frame.
        int aimBone = 0;
        // Aim at the measured joint when the pose has it on screen, and only
        // fall back to the interpolated head-to-feet line when it does not.
        bool aimUseBones = true;
        // Sticky lock: the body already locked keeps the pull unless another one
        // is this many pixels closer to the centre. Without it a crouch that
        // re-ranks the candidates for one frame swings the crosshair sideways.
        float aimSwitchMargin = 30.0f;
        // Humanisation: a fresh offset inside the hitbox every period, so the
        // pull does not land on the exact same pixel every time.
        bool aimJitter = true;
        float aimJitterPixels = 1.5f;
        int aimJitterPeriodMs = 220;
        // Skip bodies whose health has been read as zero or below.
        bool aimSkipDead = true;
        bool aimDrawTarget = true;  // box + marker on the locked entity
        // Trajectory lead: how far ahead of the target the crosshair is placed.
        // The value is the flight-time estimate in milliseconds, not a distance,
        // so a change of weapon only needs this one number.
        bool aimPrediction = true;
        float aimLeadMs = 60.0f;
        // Angle budget. Degrees are converted to pixels per frame with the
        // game's on-screen scale, which the user calibrates once.
        float aimPixelsPerDegree = 21.0f;
        float aimMaxDegreesPerSecond = 720.0f;
        // Ring at the aim radius. Off by default: it is a calibration aid, and
        // while closed the overlay should show only targets.
        bool aimFovCircle = false;

        // --- colours -------------------------------------------------------
        // 0xRRGGBB, alpha is fixed per element so brightness stays readable on
        // a bright map. Defaults are the colours the overlay shipped with.
        unsigned int colorBox = 0x78FFA0;      // 120,255,160
        unsigned int colorLocked = 0xFFBE78;   // 255,190,120 (aim-locked body)
        unsigned int colorSkeleton = 0x8CDCFF; // 140,220,255
        unsigned int colorFov = 0xFFC878;      // 255,200,120
        // One colour per side. When a body has no role yet the neutral box
        // colour is used, so an unread role never looks like a faction.
        unsigned int colorInnocent = 0x86CFA0; // 134,207,160
        unsigned int colorCriminal = 0xF07467; // 240,116,103
        unsigned int colorCop = 0x8EB4F2;      // 142,180,242
        // Text keeps its own colour so the box can be recoloured without making
        // the nickname unreadable on a bright wall.
        unsigned int colorName = 0xF2F2F4;
        unsigned int colorDistance = 0xB4B4BC;
        bool roleTags = true;

        // --- trigger -------------------------------------------------------
        // Fires the left button while the activation key is held and a target
        // sits inside the trigger radius. Same target as the aim, so it never
        // shoots at somebody the aim has not already accepted.
        bool triggerEnabled = false;
        int triggerKey = 0x06;      // VK_XBUTTON2 (mouse button 5)
        float triggerRadiusPx = 14.0f;
        int triggerDelayMs = 0;     // 0 = every frame while on target

        // --- radar ---------------------------------------------------------
        bool radarEnabled = true;
        bool radarRotate = true;    // rotate with the camera (forward is up)
        bool radarSquare = false;
        bool radarSweep = true;
        // 0 bottom-left, 1 bottom-right, 2 top-right, 3 top-left. Bottom-right is
        // the default because the game keeps its minimap top-left and chat
        // bottom-left, and a radar on either of those hides information.
        int radarCorner = 1;
        // Free placement: the centre as a fraction of the display, 0..1. Written
        // by dragging the disc and by the X/Y sliders; the corner chips are used
        // only while this is off.
        bool radarFree = false;
        float radarX = 0.86f;
        float radarY = 0.79f;
        // Held with the left button to drag the disc; VK_F8 by default. The drag
        // needs the menu open, because the game welds the OS cursor to the
        // centre of the window while it owns the mouse.
        int radarMoveKey = 0x77;
        float radarSize = 180.0f;   // diameter, screen pixels
        float radarZoom = 1.5f;     // screen pixels per world metre
        unsigned int colorRadarEnemy = 0xF07467;
        unsigned int colorRadarInnocent = 0x86CFA0;
        unsigned int colorRadarCop = 0x8EB4F2;
        unsigned int colorRadarGrid = 0x33333C;
        unsigned int colorRadarSelf = 0xF5782A;

        // --- host ----------------------------------------------------------
        // Menu toggle. INSERT by default, and the key is edge-detected, so
        // holding it does not strobe the window.
        int menuKey = 0x2D; // VK_INSERT
    };

    // Projected entity, ready to draw. The screen values are pixels with the
    // origin at the top-left, the way ImGui wants them.
    struct Entity
    {
        // Stable identity: the managed component pointer survives a walk, so a
        // pooled character keeps its velocity history across re-enumeration.
        void *key = nullptr;
        // Character root (the Animator) this entity belongs to. Several views
        // of one character share it, which is what dedupes the box and what
        // makes the self filter exact.
        void *animator = nullptr;
        int id = 0;
        // Character metadata read through the model chain: player nickname,
        // PlayerRoleDescription (citizen / robber / cop) and current armor.
        // Empty role / negative armor mean "not read this time" and are simply
        // not drawn — never a fake value.
        char nickname[40]{};
        char role[24]{};
        RoleType type = RoleType::Unknown;
        int armor = -1;
        int health = -1;         // -1 = not read this sample

        Vec3 feet{};
        Vec3 head{};
        Vec3 predictedHead{};    // head advanced by the lead time, world space
        Vec3 velocity{};        // world units per second, smoothed
        float speed = 0.0f;

        // Skeleton joints, index = UnityEngine.HumanBodyBones. boneVisible is
        // per joint: a read that failed this sample is skipped instead of
        // drawing a limb to the world origin. The values are OFFSETS from the
        // anchor below, not world positions: the pose read runs on its own
        // budget (six rigs a tick), so absolute joints from two different
        // moments made a figure in a moving car jump and drift. Offsets plus
        // one live anchor keep the whole body rigid on every frame.
        Vec3 bones[kBoneCount]{};
        // Player_Woman(Clone) has no M_head_2_jnt (skelet/жен.txt: the chain is
        // M_neck_jnt -> M_head_1_jnt -> M_hair_1_jnt), so BoneHead on that skin
        // is the skull BASE. The crown has to be lifted along the spine or the
        // head circle lands on the jaw. Carried here because the lift is a
        // property of the rig the joints came from, and both the Sample() stamp
        // and the later slot sync need it.
        bool headIsBase = false;
        // Feet position at the moment the joints were read.
        Vec3 poseAnchor{};
        // Rig root minus the model root, read on the same sample. The joints are
        // stored as offsets from `poseAnchor` (the rig root they hang from), but
        // the body position and the box are built from the model root. Drawing
        // the joints from the model root directly shifted the whole figure by
        // (rig root - model root), and that difference changes as the two run
        // out of phase - the skeleton leading the runner. Adding this offset puts
        // the joints back on their own anchor while the box stays on the ground.
        Vec3 anchorOffset{};
        // Low-passed world feet: the figure is drawn from this instead of the
        // raw sample, so a 30 Hz position read does not read as a stutter
        // against a 144 Hz camera. Time-based (tau, not a per-frame factor):
        // a constant blend is a different amount of smoothing at every frame
        // rate, which is its own flicker.
        Vec3 smoothFeet{};
        bool smoothInit = false;
        unsigned long long smoothTick = 0;
        // Throttle for the "live:" diagnostic line (raw sample vs drawn anchor).
        unsigned long long traceTick = 0;
        float boneX[kBoneCount]{};
        float boneY[kBoneCount]{};
        bool boneVisible[kBoneCount]{};
        // Projection result for this frame. Kept apart from boneVisible: a joint
        // that leaves the frustum for one frame must come back when it returns,
        // and folding both flags into one array made a joint vanish for good.
        bool boneOnScreen[kBoneCount]{};
        // Latched "this character has a drawable figure": set at 6 visible
        // joints, cleared at 3, so one budgeted joint read going missing does not
        // make the skeleton blink at range.
        bool bonesShown = false;
        // When the joints in bones[] were read, in microseconds (the stamp of
        // the capture, not of this frame). Doubles as the capture identity: the
        // same value on two frames means BonePass has not touched this body in
        // between, so the pose is the one already blended.
        unsigned long long boneTick = 0;
        // Previous capture, offsets from its own anchor, with the microsecond
        // stamp of when it was read. Joint reads land at 30-60 Hz while the
        // screen runs at 75+, so a rigid newest-pose draw steps: freeze for one
        // interval, then snap. Reproject extends the newest pose along the step
        // between these two, which turns the staircase into motion without a
        // single extra memory read.
        Vec3 bonesPrev[kBoneCount]{};
        unsigned long long bonePrevTick = 0;
        bool bonesPrevValid = false;
        // Drawn joint offsets after the temporal follow in Reproject. The
        // capture offsets above land at 30-60 Hz; adding them to a smoothed
        // anchor still steps the figure once per capture, and at a stride
        // reversal that step is backwards. This is what is actually drawn.
        Vec3 bonesSmooth[kBoneCount]{};
        bool bonesSmoothValid = false;
        unsigned long long bonesSmoothTick = 0;

        // Box quad in pixels, recomputed every frame from the projected joints.
        // Drawing straight from these instead of an estimated height ratio is
        // what keeps the rectangle on the body through crouch, prone and lean.
        float boxLeft = 0.0f;
        float boxRight = 0.0f;
        float boxTop = 0.0f;
        float boxBottom = 0.0f;
        bool boxFromBones = false;
        // First-frame guard for the box smoothing in Reproject: the quad is
        // eased towards the measured one so a one-frame joint drop cannot make
        // the rectangle jump, but a real move snaps straight through.
        bool boxInit = false;

        float feetX = 0.0f;
        float feetY = 0.0f;
        float headX = 0.0f;
        float headY = 0.0f;

        // Head position advanced by the lead time; refreshed every frame from
        // the cached velocity, so aiming stays smooth between the 2 Hz walks.
        float predictedHeadX = 0.0f;
        float predictedHeadY = 0.0f;
        float leadX = 0.0f;
        float leadY = 0.0f;

        float distance = -1.0f;
        bool visible = false;
        // True when the feet project inside the viewport plus the same margin
        // the bone pass uses to decide what it is willing to read. boneAge= is
        // measured over these bodies only: visible is just "in front of the
        // camera", and a body at the edge of the screen is deliberately not
        // refreshed, so counting it reported seconds of stale bones on a
        // healthy frame (found=7, projected=3, boneAge=20717 ms in one line).
        bool onScreen = false;
        bool locked = false;
        unsigned long long sampleTick = 0;   // when feet was read, microseconds QPC
        char name[40] = "";
    };

    // Wires the module to the live resolver and to the overlay's log sink.
    // Both must outlive the module; the overlay keeps them as file-scope statics.
    void Attach(Il2CppResolver *resolver, void (*log)(const char *message));

    Options &Config();
    // Managed class to treat as a player anchor, e.g. the character view each
    // rendered player owns. Editable in the ui; takes effect on the next tick.
    char *TargetClassName();
    char *TargetNamespace();
    float &HeadOffset();

    // Forces the next tick to rebuild the cached component list instead of
    // waiting for scanIntervalMs. Wired to the esp tab's rescan button.
    void Rescan();
    void StartWorker();

    // Resolves entities and projects them. Cheap to call every frame: the
    // managed scan runs on its own slow timer, the sample is one position read
    // per cached entity plus native matrix math.
    void Tick(float displayWidth, float displayHeight);
    // Runs the joint capture that Tick() asked for. Called right after the
    // game's own Present: Unity has finished the frame, so the rig is read at
    // the one point where the base animation clip and the weapon IK have both
    // already written their result. Reading inside the frame caught the rig
    // between the two and produced the 0.33-0.49 m pre-IK/post-IK flip-flop.
    // No-op unless Tick() set a pending capture (never in worker mode).
    void TickAfterPresent();
    // Skips the managed walk for a moment. Called when the game changes ui
    // state (menu toggled, scene loading): invoking managed code exactly then is
    // what produced the two observed hangs, so the walk stands down instead.
    void Hold(unsigned int milliseconds);
    // Draws the current list on ImGui's background draw list. Call between
    // NewFrame and Render, after Tick.
    void Draw();
    // Moves the mouse toward the closest entity inside the fov. No-op while the
    // aim key is up. deltaSeconds comes from the frame clock and only feeds the
    // angular budget.
    void Aim(float displayWidth, float displayHeight, float deltaSeconds);

    const char *Status();
    int EntityCount();
    // Radar frame: the eye, where the camera looks, and one dot per drawn body.
    // Published by Draw() so the overlay radar and the menu preview draw the
    // same snapshot instead of walking the list twice.
    struct RadarContact
    {
        float x = 0.0f;   // world metres
        float z = 0.0f;
        int type = 0;     // esp::RoleType
        float distance = 0.0f;
        float height = 0.0f;  // vertical offset in metres (dots above/below)
    };
    int RadarContacts(RadarContact *out, int capacity, float *eyeX, float *eyeZ, float *yawDegrees);
    const char *DebugLine();
    // One line of aim state for the diagnostics tab: target id, lead, step.
    const char *AimDebugLine();
}
