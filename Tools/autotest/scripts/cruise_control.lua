-- ── Параметри ────────────────────────────────────────────────────────────────
local TRIGGER_CH    = 11
local DEPTH_CH      = 10
local TRIGGER_PWM   = 1800
local ALT_TOLERANCE = 1.0
local HOLD_TIME_MS  = 5000
local UPDATE_MS     = 100

-- ── Стани ────────────────────────────────────────────────────────────────────
local STATE_IDLE    = 0
local STATE_DESCEND = 1
local STATE_HOLD    = 2
local STATE_ASCEND  = 3

-- ── Стан ─────────────────────────────────────────────────────────────────────
local state          = STATE_IDLE
local prev_trigger   = 0
local prev_depth_zone = -1
local start_alt      = 0.0
local target_alt     = 0.0
local hold_start_ms  = 0

-- ── Хелпери ──────────────────────────────────────────────────────────────────
local function get_depth_and_notify()
    local pwm = rc:get_pwm(DEPTH_CH)
    if not pwm then return 20.0 end

    local depth, zone
    if pwm < 1250 then
        depth, zone = 20.0, 0
    elseif pwm < 1500 then
        depth, zone = 50.0, 1
    elseif pwm < 1750 then
        depth, zone = 100.0, 2
    else
        depth, zone = 150.0, 3
    end

    if zone ~= prev_depth_zone then
        prev_depth_zone = zone
        gcs:send_text(3, string.format("DEPTH: %.0fm (RC10=%d)", depth, pwm))
    end

    return depth
end

local function set_alt(alt_m)
    local current = baro:get_altitude()
    local err = alt_m - current
    local vz = math.max(-3.0, math.min(3.0, -err * 0.5))
    local vel = Vector3f()
    vel:x(0)
    vel:y(0)
    vel:z(vz)
    vehicle:set_target_velocity_NED(vel)
end

-- ── Основний цикл ────────────────────────────────────────────────────────────
local function update()
    local trigger_pwm = rc:get_pwm(TRIGGER_CH)
    if not trigger_pwm then return update, UPDATE_MS end

    local trigger     = trigger_pwm > TRIGGER_PWM and 1 or 0
    local current_alt = baro:get_altitude()
    local depth       = get_depth_and_notify()

    -- тригер старту
    if trigger == 1 and prev_trigger == 0 and state == STATE_IDLE then
        start_alt  = current_alt
        target_alt = current_alt - depth
        vehicle:set_mode(4)  -- GUIDED
        state = STATE_DESCEND
        gcs:send_text(3, string.format("DIVE: %.1fm -> %.1fm (depth %.0fm)",
            start_alt, target_alt, depth))
    end

    prev_trigger = trigger

    -- ── Стейт машина ─────────────────────────────────────────────────────────
    if state == STATE_DESCEND then
        set_alt(target_alt)
        gcs:send_text(6, string.format("DESC alt=%.1f tgt=%.1f", current_alt, target_alt))
        if math.abs(current_alt - target_alt) <= ALT_TOLERANCE then
            hold_start_ms = millis():toint()
            state = STATE_HOLD
            gcs:send_text(3, string.format("DIVE: hold 5s at %.1fm", current_alt))
        end

    elseif state == STATE_HOLD then
        set_alt(target_alt)
        if millis():toint() - hold_start_ms >= HOLD_TIME_MS then
            state = STATE_ASCEND
            gcs:send_text(3, string.format("DIVE: ascending to %.1fm", start_alt))
        end

    elseif state == STATE_ASCEND then
        set_alt(start_alt)
        gcs:send_text(6, string.format("ASCND alt=%.1f tgt=%.1f", current_alt, start_alt))
        if math.abs(current_alt - start_alt) <= ALT_TOLERANCE then
            state = STATE_IDLE
            vehicle:set_mode(5)  -- LOITER
            gcs:send_text(3, "DIVE: complete -> LOITER")
        end
    end

    return update, UPDATE_MS
end

return update, UPDATE_MS