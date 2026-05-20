-- rc_failsafe_loiter.lua
-- Intercepts ArduPilot RC failsafe LAND -> LOITER (or ALT_HOLD fallback)

gcs:send_text(6, "FS: interceptor v5.2 loaded")

local MODE_LOITER      = 5
local MODE_ALT_HOLD    = 2
local MODE_LAND        = 9
local UPDATE_MS        = 200
local HEARTBEAT_MS     = 10000
local MODE_SET_WAIT_MS = 400
local INTERCEPT_COOLDOWN = 2000

local last_heartbeat        = 0
local mode_set_at           = 0
local pending_mode          = nil
local intercept_cooldown_ms = 0

local function try_safe_hold()
  if not vehicle:set_mode(MODE_LOITER) then
    vehicle:set_mode(MODE_ALT_HOLD)
    -- gcs:send_text(3, "FS: LOITER unavailable -> ALT_HOLD")
    mode_set_at  = millis()
    pending_mode = MODE_ALT_HOLD
  else
    mode_set_at  = millis()
    pending_mode = MODE_LOITER
  end
end

function update()
  local now = millis()
  if not now then return update, UPDATE_MS end

  -- heartbeat
  if (now - last_heartbeat) >= HEARTBEAT_MS then
    last_heartbeat = now
    -- gcs:send_text(6, "FS: interceptor active, armed=" .. tostring(arming:is_armed()))
  end

  if not arming:is_armed() then
    pending_mode            = nil
    mode_set_at             = 0
    intercept_cooldown_ms   = 0
    return update, UPDATE_MS
  end

  -- check pending mode_set result
  if pending_mode == MODE_LOITER and (now - mode_set_at) >= MODE_SET_WAIT_MS then
    if vehicle:get_mode() ~= MODE_LOITER then
      vehicle:set_mode(MODE_ALT_HOLD)
      -- gcs:send_text(3, "FS: LOITER failed -> ALT_HOLD")
    end
    pending_mode = nil
  end

  -- intercept LAND with cooldown
  if vehicle:get_mode() == MODE_LAND then
    if (now - intercept_cooldown_ms) >= INTERCEPT_COOLDOWN then
      intercept_cooldown_ms = now
      try_safe_hold()
      -- gcs:send_text(3, "FS: LAND intercepted -> LOITER")
    end
  end

  return update, UPDATE_MS
end

return update()