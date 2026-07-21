-- rc_failsafe_poshold.lua
-- Intercepts ArduPilot RC failsafe LAND -> POSHOLD (or ALT_HOLD fallback)

gcs:send_text(6, "FS: LOITER v1 loaded")

local MODE_POSHOLD           = 16
local MODE_ALT_HOLD          = 2
local MODE_LAND              = 9
local MODE_REASON_RADIO_FS   = 3   -- ModeReason::RADIO_FAILSAFE
local UPDATE_MS              = 200
local HEARTBEAT_MS           = 10000
local MODE_SET_WAIT_MS       = 400
local INTERCEPT_COOLDOWN     = 2000

local last_heartbeat        = 0
local mode_set_at           = 0
local pending_mode          = nil
local intercept_cooldown_ms = 0

local function try_safe_hold()
  if not vehicle:set_mode(MODE_POSHOLD) then
    vehicle:set_mode(MODE_ALT_HOLD)
    gcs:send_text(3, "FS: POSHOLD unavailable -> ALT_HOLD")
    mode_set_at  = millis()
    pending_mode = MODE_ALT_HOLD
  else
    mode_set_at  = millis()
    pending_mode = MODE_POSHOLD
  end
end

function update()
  local now = millis()
  if not now then return update, UPDATE_MS end


  if not arming:is_armed() then
    pending_mode          = nil
    mode_set_at           = 0
    intercept_cooldown_ms = 0
    return update, UPDATE_MS
  end

  if pending_mode == MODE_POSHOLD and (now - mode_set_at) >= MODE_SET_WAIT_MS then
    if vehicle:get_mode() ~= MODE_POSHOLD then
      vehicle:set_mode(MODE_ALT_HOLD)
      gcs:send_text(3, "FS: POSHOLD failed -> ALT_HOLD")
    end
    pending_mode = nil
  end

  -- intercept LAND only when it was caused by RC (radio) failsafe
  if vehicle:get_mode() == MODE_LAND and vehicle:get_control_mode_reason() == MODE_REASON_RADIO_FS then
    if (now - intercept_cooldown_ms) >= INTERCEPT_COOLDOWN then
      intercept_cooldown_ms = now
      try_safe_hold()
      gcs:send_text(3, "FS: RC failsafe LAND intercepted -> POSHOLD")
    end
  end

  return update, UPDATE_MS
end

return update()