--[[
PTH (Samamet) Serial Sensor Reading

Read the data from the serial line that the PTH is connected to, then decode the
messages from it, and log the data to the autopilots BIN file.

Author: Justin Tussey
Last Updated: 2024-05-24
]] --

-- variable to count iterations without getting message
local loops_since_data_received = 0

-- table to hold the message that is currently being assembled
local message_table = {}

-- error type table
local ERROR_LIST = {
  "No data received",      -- 1
  "Checksum fail",         -- 2
  "Data parsing fail",     -- 3
}

-- initialize serial connection
local BAUD_RATE = 9600

-- find the serial first (0) scripting serial port instance
-- SERIALx_PROTOCOL 28
local PORT = assert(serial:find_serial(0),"Could not find Scripting Serial Port")

-- begin the serial port
PORT:begin(BAUD_RATE)
PORT:set_flow_control(0)

-- Calculate checksum by taking string from '$' to '*' (but not including them),
-- then XORing each of the ASCII characters in the string with the next one in
-- the string. Then compare the result in hex with the 2 character hex code
-- after the "*"
function verify_checksum(message_string)
  -- take the sub-string from (but not including) "$" and "*"
  local data_string = message_string:match("%$(.*)%*")

  -- check that the regex successfully parsed the string
  if data_string == nil then
    return false
  end

  -- extracts the the two characters after the '*' in the message string, and
  -- only accepts valid "2 character" hex numbers, ie: 4A. Then take that string
  -- then convert it to an integer (16 specifies that our input string is a hex
  -- number)
  local incoming_checksum = message_string:match("%*([0-9A-F][0-9A-F])")

  -- check that the regex successfully parsed the string
  if incoming_checksum == nil then
    return false
  end

  incoming_checksum = tonumber(incoming_checksum, 16)

  -- starting value of zeros, which will not effect the first XOR
  local checksum = 0x0

  -- Take the message string and convert it to bytes, then place into curly
  -- braces to convert it to an array of bytes, where each byte is an ASCII
  -- character. This allows us to iterate over it and do bitwise operations on
  -- it (lua does not support bitwise ops on strings/chars)
  local string_bytes = { data_string:byte(1, #data_string) }

  -- Iterate over the character array, and XOR each character with the
  -- previous "summed" XOR operations
  for i = 1, #string_bytes do
    checksum = (checksum ~ string_bytes[i])
  end

  -- check if checksum matches, return true or false
  return (checksum == incoming_checksum)

end

-- Take the data from the verified message string and split up the string, using
-- a comma as a delimiter. Then take the measurements from the data and log them
-- to the BIN file.
function parse_data(message_string)
  -- take the sub-string from (but not including) "$" and "*"
  local data_string = message_string:match("%$(.*)%*")

  -- check that the regex successfully parsed the string
  -- (unlikely since we already checked but it doesn't hurt)
  if data_string == nil then
    return false
  end

  local data_table = {}

  -- take the string, match up until the first comma, place that substring
  -- into the table, then repeat for the rest of the string.
  -- The regex is matching all characters up until it finds a ','. The "+" at
  -- the end allows matching more than once
  for str in string.gmatch(data_string, "([^" ..",".. "]+)") do
    table.insert(data_table, str)
  end

  -- make sure regex successfully splits by commas.
  -- (hopefully will not reach here since the checksum should fail before then)
  if #data_table ~= 12 then
    return false
  end

  local measurements_table={}
  for i=3,12,2 do
    table.insert(measurements_table, data_table[i])
  end

-- report data to Mission Planner, not necessary all the time
  gcs:send_text(7, "pres:" .. string.format(" %.2f \r\n", measurements_table[1]) ..
                   "temp1:" .. string.format(" %.2f \r\n", measurements_table[2]) ..
                   "temp2:" .. string.format(" %.2f \r\n", measurements_table[3]) ..
                   "hum:" .. string.format(" %.2f \r\n", measurements_table[4]) ..
                   "temp3:" .. string.format(" %.2f", measurements_table[5])
  )

  -- return whether data input data matched needed format (table with 5
  -- elements)
  return log_data(measurements_table)

end

-- take the input table, and check if it has 5 elements in it, then write that
-- data to the BIN file, with the appropriate names and units
function log_data(measurements_table)
  -- care must be taken when selecting a name, must be less than four characters and not clash with an existing log type
  -- format characters specify the type of variable to be logged, see AP_Logger/README.md
  -- https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Logger
  -- not all format types are supported by scripting only: i, L, e, f, n, M, B, I, E, and N
  -- Data MUST be integer|number|uint32_t_ud|string , type to match format string
  -- lua automatically adds a timestamp in micro seconds
  if #measurements_table ~= 5 then
    return false
  end
  logger:write('SAMA', 'pres,temp1,temp2,hum,temp3,error', -- section name and labels
               'NNNNNN',                              -- data type (char[16])
               'POO%O-',                              -- units,(Pa, C, C, % (humidity), C)
               '------',                              -- multipliers (- signifies no multiplier)
               measurements_table[1],                -- data for labels
               measurements_table[2],
               measurements_table[3],
               measurements_table[4],
               measurements_table[5],
               "Normal")
  return true
end

-- called when an error is detected, and writes all zeros to the BIN file
function log_error(error_type)
  logger:write('SAMA', 'pres,temp1,temp2,hum,temp3,error', -- section name and labels
               'NNNNNN',                                   -- data type (char[16])
               'POO%O-',                                   -- units,(Pa, C, C, % (humidity), C)
               '------',                                   -- multipliers (- signifies no multiplier)
               '0', '0', '0', '0', '0', error_type)        -- zeros for labels since there is an error with the PTH or its data
end


function update()
  local n_bytes = PORT:available()

  -- If we have received no bytes or have not received any new bytes, increment
  -- the count of loops without data. If it reaches 11 or more
  -- (100ms * 11 = 1.1sec), then log an error.
  if n_bytes <= 0 then
    loops_since_data_received = loops_since_data_received + 1
    if loops_since_data_received >= 11 then
      log_error(ERROR_LIST[1])
      -- Send error message to mission planner with priority 0 (error)
      gcs:send_text(0, "ERROR: PTH has failed to send data")
      -- clear incomplete message (if there is one)
      message_table = {}
    end
    return update, 100
  end

  -- read bytes from the serial line until we hit '0x0A' which is <LF>, the
  -- ending of the message. Then process the message
  loops_since_data_received = 0
  while n_bytes > 0 do
    local byte = PORT:read()
    if byte == 0x0A then
      table.insert(message_table, byte)
      local message_string = string.char(table.unpack(message_table))
      if not (verify_checksum(message_string)) then
        log_error(ERROR_LIST[2])
        gcs:send_text(0, "ERROR: PTH Data failed checksum")
        message_table = {}
        return update, 100
      end
      if not (parse_data(message_string)) then
        log_error(ERROR_LIST[3])
        gcs:send_text(0, "ERROR: Failed to parse data")
        message_table = {}
        return update, 100
      end
      -- reset for the next message
      message_table = {}
      return update, 100
    end
    table.insert(message_table, byte)
    n_bytes = n_bytes - 1
  end

  return update, 100 -- reschedules the loop for every 100ms
end

return update() -- run immediately before starting to reschedule
