-- rc_failsafe_loiter.lua
-- LAND (failsafe trigger) -> gradual attitude brake in GUIDED_NOGPS -> LOITER

gcs:send_text(6, "FS: brake interceptor v6.1 loaded")

local MODE_LOITER      = 5
local MODE_GUIDED_NOGPS = 20
local MODE_LAND        = 9
local UPDATE_MS        = 200

-- brake tuning
local BRAKE_STEP_DEG   = 4.0   -- deg per tick (200ms) = 20 deg/s reduction rate
local SPEED_THRESH_MS  = 1.2   -- m/s horizontal: safe to enter LOITER
local LEVEL_THRESH_DEG = 2.5   -- deg: consider attitude zeroed

-- cooldown between intercept attempts
local INTERCEPT_COOLDOWN_MS = 3000

local intercept_cooldown_ms = 0

-- braking state
local braking      = false
local target_roll  = 0
local target_pitch = 0
local hold_yaw     = 0

local function step_to_zero(val, step)
  if val > step then
    return val - step
  elseif val < -step then
    return val + step
  end
  return 0
end

local function start_brake()
  hold_yaw     = math.deg(ahrs:get_yaw())
  target_roll  = math.deg(ahrs:get_roll())
  target_pitch = math.deg(ahrs:get_pitch())

  -- switch mode and immediately override the level init state
  -- both calls happen in the same Lua tick so flight controller
  -- never sees the zero-attitude from angle_control_start()
  if vehicle:set_mode(MODE_GUIDED_NOGPS) then
    vehicle:set_target_angle_and_climbrate(
      target_roll, target_pitch, hold_yaw, 0, false, 0)
    braking = true
    gcs:send_text(3, string.format(
      "FS: brake start R=%.1f P=%.1f", target_roll, target_pitch))
  else
    -- GUIDED_NOGPS unavailable, fall back to LOITER directly
    vehicle:set_mode(MODE_LOITER)
    gcs:send_text(3, "FS: GUIDED_NOGPS failed, direct LOITER")
  end
end

local function brake_step(now)
  -- RC restored
  if rc:has_valid_input() then
    vehicle:set_mode(MODE_LOITER)
    braking = false
    intercept_cooldown_ms = now
    gcs:send_text(6, "FS: RC restored -> LOITER")
    return
  end

  -- ArduPilot re-triggered LAND while we are braking: push back to GUIDED_NOGPS
  if vehicle:get_mode() == MODE_LAND then
    vehicle:set_mode(MODE_GUIDED_NOGPS)
  end

  target_roll  = step_to_zero(target_roll,  BRAKE_STEP_DEG)
  target_pitch = step_to_zero(target_pitch, BRAKE_STEP_DEG)

  vehicle:set_target_angle_and_climbrate(
    target_roll, target_pitch, hold_yaw, 0, false, 0)

  -- switch to LOITER only when actually slow
  local spd = ahrs:groundspeed_vector()
  local speed = spd and spd:length() or 999

  local attitude_zeroed = math.abs(target_roll)  < LEVEL_THRESH_DEG
                       and math.abs(target_pitch) < LEVEL_THRESH_DEG

  if attitude_zeroed and speed < SPEED_THRESH_MS then
    vehicle:set_mode(MODE_LOITER)
    braking = false
    intercept_cooldown_ms = now
    gcs:send_text(6, string.format(
      "FS: braked -> LOITER (spd=%.1f)", speed))
  end
end

function update()
  local now = millis()

  if not arming:is_armed() then
    braking = false
    intercept_cooldown_ms = 0
    return update, UPDATE_MS
  end

  if braking then
    brake_step(now)
    return update, UPDATE_MS
  end

  -- intercept LAND with cooldown
  if vehicle:get_mode() == MODE_LAND then
    if (now - intercept_cooldown_ms) >= INTERCEPT_COOLDOWN_MS then
      intercept_cooldown_ms = now
      start_brake()
    end
  end

  return update, UPDATE_MS
end

return update()
