--[[

I2C TCA9548 interface

This program uses the TCA Multiplexer to communicate to multiple I2C devices
with the same addresses

Author: Ryan Prince | Last Updated By: Justin Tussey
Last Updated: 2024-06-04

]] --

-- init i2c bus
local I2C_BUS = i2c:get_device(0, 0)

-- make sure that get_device does not that return nil
if (I2C_BUS == nil) then
  gcs:send_text(0, "Cannot find I2C bus")
  return
end

I2C_BUS:set_retries(10)
gcs:send_text(7, "i2c_tca Script Started!")

-- var for address of the sensors
local SENSOR_ADDR = 0x28

-- table of which channels on the multiplexer are connected
local CHANNEL_NUMBERS = {
  0,
  1,
  2,
  3,
  4
}

-- for each TCA9548A, add an entry with its address
-- 0x70 is default, to add more set or reset A0, A1, A2
local TCA_ADDRESS = 0x70

-- list for the log data from the sensors
local log_data_list = {}

-- list for errors when reading channels of multiplexer
local error_list = {}

-- opens the channel to the designated TCA module
local function tcaselect(channel)
  -- set multiplexer address
  I2C_BUS:set_address(TCA_ADDRESS)

  -- make sure channel value passed through is between 0-7
  if (channel > 7) or (channel < 0) then
    return false
  end

  -- set/open the correct channel
  -- i2c_bus:write_register(0x70, 1 << channel)
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
local function log_channel_error(channel_index)
  log_data_list[channel_index] = "0"
  error_list[channel_index] = "ERROR"
end

-- MAIN FUNCTION
function update()
  for key, value in pairs(CHANNEL_NUMBERS) do

    -- select channel i on TCA
    if not (tcaselect(value)) then
      gcs:send_text(0, "Error when selecting tube " .. tostring(key))
      log_channel_error(key)
    else
      -- open the address of the sensor
      I2C_BUS:set_address(SENSOR_ADDR)

      -- read_registers(begin at register, number of bytes to read)
      local returnTable = I2C_BUS:read_registers(0, 2)

      -- if there is no i2c device connected (or no data is read in general) log it as an error
      if (returnTable == nil) then
        gcs:send_text(0, "sensor disconnected, " .. " tube: " .. tostring(key))
        log_channel_error(key)
      else

        -- format data to remove first 2 bits
        local msg = (returnTable[1] << 8 | returnTable[2]) & 0x3FFF

        -- normalize data to [-2 2] in inH2O and make the datatype string
        -- math is ((range*data)/max(data) - 2)
        local normalized_data = tostring((4.0 * msg) / 0x3FFF - 2)
        log_data_list[key] = normalized_data
        error_list[key] = "NORMAL"
      end
    end
  end

  log_data()

  -- send_text(priority level (7 is Debug), text as a string formatted to float)
  -- report data to mission planner output
  gcs:send_text(7, "tube1 " .. string.format(": %.2f | ", log_data_list[1]) ..
                   "tube2 " .. string.format(": %.2f | ", log_data_list[2]) ..
                   "tube3 " .. string.format(": %.2f | ", log_data_list[3]) ..
                   "tube4 " .. string.format(": %.2f | ", log_data_list[4]) ..
                   "tube5 " .. string.format(": %.2f", log_data_list[5])
  )

  I2C_BUS:set_address(0x00)
  return update, 50 -- reschedules the loop every 50ms (20hz)
end

return update() -- run immediately before starting to reschedule
