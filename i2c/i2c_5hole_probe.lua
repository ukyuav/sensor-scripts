--[[

I2C 5 Hole Probe w/ TCA9548 Multiplexer interface

This script uses an I2C multiplexer to communicate to multiple I2C pressure
sensors with the same addresses to record pressure data from a 5 hole pitot tube
probe onto the drone's autopilot for turbulence data collection

Author: Ryan Prince | Last Updated By: Justin Tussey
Last Updated: 2024-06-10

]] --

-- Global Constants --

-- init i2c bus
-- Get interface at bus 0 (first I2C bus) and set device address to 0x0
local I2C_BUS = i2c:get_device(0, 0)

-- -- make sure that get_device does not return nil
-- if (I2C_BUS == nil) then
--   gcs:send_text(0, "Cannot find I2C bus")
--   return
-- end

-- set the number of retries to 10
I2C_BUS:set_retries(10)
gcs:send_text(7, "i2c_tca Script Started!")

-- shared address of the sensors
local SENSOR_ADDR = 0x28

-- table of which channels on the multiplexer are being used
local CHANNEL_NUMBERS = {
  0,
  1,
  2,
  3,
  4
}

-- error type table
local ERROR_LIST = {
  "Sensor Disconnected (sensor fail)",          -- 1
  "Failed to switch channel (multiplexer fail)" -- 2
}

-- 0x70 is default, to change, set or reset A0, A1, A2 on the multiplexer
local TCA_ADDRESS = 0x70


-- Global Variables --

-- list for the log data from the sensors
local log_data_list = {}

-- list for errors when reading channels of multiplexer
local error_list = {}


-- dynamically create the message that gets reported to mission planner
-- prevents us from having to manually change the message form every time we add
-- or remove sensors or decide to change the format of the message
local function form_message()
  local message = ""
  for key, value in pairs(CHANNEL_NUMBERS) do
    message = message .. string.format(key) .. string.format(": %.2f ", log_data_list[key])
  end
  return message
end

-- set the current channel on the TCA
local function tcaselect(channel)
  -- set multiplexer address
  I2C_BUS:set_address(TCA_ADDRESS)

  -- make sure channel value passed through is between 0-7
  if (channel > 7) or (channel < 0) then
    return false
  end

  -- set/open the correct channel
  return (I2C_BUS:write_register(TCA_ADDRESS, 1 << channel))
end

-- write the data to the autopilot's BIN file
local function log_data()
  -- care must be taken when selecting a name, must be less than four characters and not clash with an existing log type
  -- format characters specify the type of variable to be logged, see AP_Logger/README.md
  -- https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger
  -- not all format types are supported by scripting only: i, L, e, f, n, M, B, I, E, and N
  -- Data MUST be integer|number|uint32_t_ud|string , type to match format string
  -- lua automatically adds a timestamp in micro seconds
  logger:write('PRBE','tube1,tube2,tube3,tube4,tube5,err1,err2,err3,err4,err5','NNNNNNNNNN',
               log_data_list[1], log_data_list[2], log_data_list[3], log_data_list[4], log_data_list[5],
               error_list[1], error_list[2], error_list[3], error_list[4], error_list[5])
end

-- write an error to the channel that is experience an error
local function log_channel_error(channel_index, error_type)
  log_data_list[channel_index] = "0"
  error_list[channel_index] = error_type
end

-- MAIN FUNCTION
function update()
  -- go through each channel, and collect the data from the sensor
  for key, value in pairs(CHANNEL_NUMBERS) do

    -- select channel i on TCA
    if not (tcaselect(value)) then
      gcs:send_text(0, "Error when selecting tube " .. tostring(key))
      log_channel_error(key, ERROR_LIST[1])
    else
      -- open the address of the sensor
      I2C_BUS:set_address(SENSOR_ADDR)

      -- read_registers(begin at register, number of bytes to read)
      local returnTable = I2C_BUS:read_registers(0, 2)

      -- if there is no i2c device connected (or no data is read in general) log it as an error
      if (returnTable == nil) then
        gcs:send_text(0, "sensor disconnected, " .. " tube: " .. tostring(key))
        log_channel_error(key, ERROR_LIST[2])
      else

        -- format data to remove first 2 bits
        local msg = (returnTable[1] << 8 | returnTable[2]) & 0x3FFF

        -- normalize data to [-2 2] in inH2O and make the datatype string
        -- math is ((range*data)/max(data) - 2)
        local normalized_data = tostring((4.0 * msg) / 0x3FFF - 2)
        -- add the data to the list
        log_data_list[key] = normalized_data
        error_list[key] = "NORMAL"
      end
    end
  end

  log_data()


  -- send_text(priority level (7 is Debug), text is formed dynamically from the function)
  gcs:send_text(7, form_message())

  -- report data to mission planner output
  -- gcs:send_text(7, "1 " .. string.format(": %.2f | ", log_data_list[1]) ..
  --                  "2 " .. string.format(": %.2f | ", log_data_list[2]) ..
  --                  "3 " .. string.format(": %.2f | ", log_data_list[3]) ..
  --                  "4 " .. string.format(": %.2f | ", log_data_list[4]) ..
  --                  "5 " .. string.format(": %.2f", log_data_list[5])
  -- )


  -- reset everything for the next loop
  I2C_BUS:set_address(0x00)
  log_data_list = {}
  error_list = {}
  return update, 50 -- reschedules the loop every 50ms (20hz)
end

return update() -- run immediately before starting to reschedule

