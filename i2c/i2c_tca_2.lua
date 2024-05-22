--[[

I2C TCA9548 interface

This program uses the TCA Multiplexer to communicate to multiple devices with
the same addresses

Author: Ryan Prince | Last Updated By: Justin Tussey
Last Updated: 17 May 2024

]] --

-- list for the log data from the sensors
local log_data_list = {}

-- list for errors when reading channels of multiplexer
local error_list = {}

-- init i2c bus
local i2c_bus = i2c:get_device(0, 0)
i2c_bus:set_retries(10)
gcs:send_text(7, "i2c_tca Script Started!")

-- var for address of the sensors
local SENSOR_ADDR = 0x28

-- var for list of which channels on the multiplexer are connected
local CHANNEL_NUMBERS = {
  0,
  1,
  2
}

-- for each TCA9548A, add an entry with its address
-- 0x70 is default, to add more set or reset A0, A1, A2
TCA_ADDRESSES = {
  0x70
}

-- opens the channel to the designated TCA module
function tcaselect(tca, channel)
  -- verify that tca index passed through is valid
  if (tca > #TCA_ADDRESSES) or (tca < 0) then
    return false
  end

  -- choose multiplexer from array
  i2c_bus:set_address(TCA_ADDRESSES[tca])

  -- make sure channel value passed through is between 0-7
  if (channel > 7) or (channel < 0) then
    return false
  end

  -- set/open the correct channel
  -- i2c_bus:write_register(0x70, 1 << channel)
  i2c_bus:write_register(TCA_ADDRESSES[tca], 1 << channel)
  return true
end

function log_data()
  -- care must be taken when selecting a name, must be less than four characters and not clash with an existing log type
  -- format characters specify the type of variable to be logged, see AP_Logger/README.md
  -- https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger
  -- not all format types are supported by scripting only: i, L, e, f, n, M, B, I, E, and N
  -- Data MUST be integer|number|uint32_t_ud|string , type to match format string
  -- lua automatically adds a timestamp in micro seconds
  logger:write('SENS','s1,s2,s3,err1,err2,err3','NNNNNN', log_data_list[1], log_data_list[2], log_data_list[3], error_list[1], error_list[2], error_list[3])
end

function log_channel_error(channel_index)
  log_data_list[channel_index] = "0"
  error_list[channel_index] = "ERROR"
end


-- MAIN FUNCTION
function update()
  for key, value in pairs(CHANNEL_NUMBERS) do

    -- select TCA module 1, and channel i
    if not (tcaselect(1, value)) then
      gcs:send_text(0, "Called TCA channel " .. tostring(value) .. ", which does not exist")
      log_channel_error(key)
    else
      -- once open use the address of the sensor
      if value == 0 then
        i2c_bus:set_address(0x68)
      elseif value == 1 then
        i2c_bus:set_address(0x38)
      elseif value == 2 then
        i2c_bus:set_address(0x48)
      end
      -- read_registers(begin at register, number of bytes to read)
      returnTable = i2c_bus:read_registers(0, 2)

      -- if there is no i2c device connected (or no data is read in general) log it as an error
      if (returnTable == nil) then
        gcs:send_text(0, "returnTable val nil," .. " disconn sensor," .. " channel: " .. string.format("%d", value))
        log_channel_error(key)
      else
        -- output data to MP Messages
        -- format data to remove first 2 bits
        msg = (returnTable[1] << 8 | returnTable[2]) & 0x3FFF

        -- normalize data to [-2 2] in inH2O and make the datatype string
        -- math is ((range*data)/max(data) - 2)
        normalized_data = tostring((4.0 * msg) / 0x3FFF - 2)
        log_data_list[key] = normalized_data
        error_list[key] = "NORMAL"
      end
    end
  end

  log_data()
  -- gcs:send_text(7, tostring(channel_numbers[1]) .. " | " .. tostring(log_data_list[1]))
  -- gcs:send_text(7, tostring(channel_numbers[2]) .. " | " .. tostring(log_data_list[2]))


  -- send_text(priority level (7 is Debug), text as a string formatted to float)
  -- report data to misson planner output
  gcs:send_text(7, "chan " .. string.format("%d: %.3f | ", CHANNEL_NUMBERS[1], log_data_list[1]) ..
                   "chan " .. string.format("%d: %.3f | ", CHANNEL_NUMBERS[2], log_data_list[2]) ..
                   "chan " .. string.format("%d: %.3f ", CHANNEL_NUMBERS[3], log_data_list[3])
  )

  i2c_bus:set_address(0x00)
  return update, 50 -- reschedules the loop every 50ms (20hz)
end

return update() -- run immediately before starting to reschedule
