--[[

I2C Sensor reading

This program communicates with all on board airspeed sensors and reports values in the .bin file and (optional) a csv file.

Author: Ryan Prince
Last Updated: 29 Feb 2024

]]--
-- init file and data
local file_name = "SENSOR_DATA.csv"
local log_data = {}

-- init i2c bus
local i2c_bus = i2c:get_device(0, 0x00)
i2c_bus:set_retries(10)
gcs:send_text(7, "i2c_readSensors Script Started!")

-- init addresses of active sensors (change to known values)
-- sensor_addr = {0x28, 0x38, 0x48, 0x58, 0x68, 0x78}
sensor_addr = {0x28, 0x38, 0x48, 0x68}

-- Function to write all data to file
local function write_to_file()
  -- open file to write to in "append" mode
  file = io.open(file_name, "a")
  if not file then
    error("Could not open file")
  end

  -- write data
  -- separate with comas and add a carriage return
  file:write(tostring(millis()) .. ", " .. table.concat(log_data,", ") .. "\n")

  -- make sure file is upto date
  file:flush()

  -- close file
  file = io.close()
end

-- write data to bin
local function write_to_dataflash()

  -- care must be taken when selecting a name, must be less than four characters and not clash with an existing log type
  -- format characters specify the type of variable to be logged, see AP_Logger/README.md
  -- https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger
  -- not all format types are supported by scripting only: i, L, e, f, n, M, B, I, E, and N
  -- Data MUST be integer|number|uint32_t_ud|string , type to match format string
  -- lua automatically adds a timestamp in micro seconds
  -- logger:write('SENS','s1,s2,s3,s4,s5,s6','NNNNNN', log_data[1], log_data[2], log_data[3], log_data[4], log_data[5], log_data[6])
  logger:write('SENS','s1,s2,s3,s4','NNNN', log_data[1], log_data[2], log_data[3], log_data[4])
end

-- MAIN FUNCTION
function update()

  -- foreach entry in sensor_addr (key = index)
  for key, value in pairs(sensor_addr) do
  i2c_bus:set_address(sensor_addr[key])

    -- make sure sensor responds
    if i2c_bus:read_registers(0) then
      -- read_registers(begin at register, number of bytes to read)
      returnTable = i2c_bus:read_registers(0, 2)

      -- output data to MP Messages
      -- format data to remove first 2 bits
      msg =  (returnTable[1] << 8 | returnTable[2]) & 0x3FFF
      -- send_text(priority level (7 is Debug), text as a string formatted to hex)
      gcs:send_text(7, "Data on " .. "0x" .. string.format("%x", sensor_addr[key]) .. ": " .. string.format("%x", msg)) -- comment out to stop console log

      -- normalize data to [-2 2] in inH2O and make the datatype string
      -- math is ((range*data)/max(data) - 2)
      log_data[key] = tostring((4.0*msg)/0x3FFF - 2)

    end
  end
  -- write data to file
  --write_to_file()

  -- write data to bin
  write_to_dataflash()

  -- reset address index
  i2c_bus:set_address(0x00)
  return update, 200 -- reschedules the loop every 200ms (~5Hz sample)
end

-- This section runs once

-- open file to write to in "append" mode
file = io.open(file_name, "a")
-- write the header in the file
--file:write('Time Stamp(ms), Sensor 1 (0x28), Sensor 2 (0x38), Sensor 3 (0x48), Sensor 4 (0x58), Sensor 5 (0x68), Sensor 6 (0x78)\n')
-- make sure file is up to date
file:flush()
-- close file
file = io.close()

return update() -- run immediately before starting to reschedule
