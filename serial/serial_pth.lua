--[[
PTH (Samamet) Serial Sensor Reading

Read the data from the serial line that the PTH is connected to, then decode the
messages from it, and log the data to the autopilots BIN file.

Author: Justin Tussey
Last Updated: 2024-05-20
]] --

-- variable to count iterations without getting message
local loops_since_data_received = 0

-- variables to count number of bytes available on serial line
local previous_n_bytes = 0
local n_bytes = 0


-- error type table
local error_list = {
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

  if checksum ~= incoming_checksum then
    return false
  else
    return true
  end

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

  -- return whether data input data matched needed format (table with 5
  -- elements)
  return log_data(measurements_table)

-- report data to Mission Planner, not necessary all the time
--   gcs:send_text(7, "\r\npres:" .. string.format(" %.2f \r\n", measurements_table[1]) ..
--                    "temp1:" .. string.format(" %.2f \r\n", measurements_table[2]) ..
--                    "temp2:" .. string.format(" %.2f \r\n", measurements_table[2]) ..
--                    "hum:" .. string.format(" %.2f \r\n", measurements_table[2]) ..
--                    "temp3:" .. string.format(" %.2f \r\n", measurements_table[2])
--   )

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
  previous_n_bytes = n_bytes
  n_bytes = PORT:available()

  -- If we have received no bytes or have not received any new bytes, increment
  -- the count of loops without data. If it reaches 6 or more
  -- (250ms * 6 = 1.5sec), then log an error.
  if (n_bytes <= 0) or (n_bytes == previous_n_bytes) then
    loops_since_data_received = loops_since_data_received + 1
    if loops_since_data_received >= 6 then
      log_error(error_list[1])
      gcs:send_text(0, "ERROR: PTH has failed to send data")
      gcs:send_text(0, "Bytes received: " .. tostring(n_bytes))
    end
      return update, 250
  else
    loops_since_data_received = 0
  end

  -- If we are in the middle of recieving a message,
  -- simply wait for the rest of the message to arrive
  while (n_bytes > 0 and n_bytes < 60)  do
    return update, 250
  end

  while n_bytes > 0 do
    if n_bytes >= 120 then
      gcs:send_text(7, "Bytes available: " .. tostring(n_bytes))
    end
    -- only read a max of 120 bytes in a go
    -- this limits memory consumption
    local buffer = {} -- table to buffer data
    local bytes_target = n_bytes - math.min(n_bytes, 60)
    while n_bytes > bytes_target do
      table.insert(buffer,PORT:read())
      n_bytes = n_bytes - 1
    end

    local data = string.char(table.unpack(buffer))

    -- check if checksum is valid
    if (verify_checksum(data)) then
      -- make sure that data is logged correctly
      if not (parse_data(data)) then
        log_error(error_list[3])
        gcs:send_text(0, "ERROR: PTH data was not successfully parsed or not written to BIN file correctly!")
        gcs:send_text(0, "Incoming string: " .. data .. string.format(" size: %d", #data))
      end
    else
      log_error(error_list[2])
      gcs:send_text(0, "ERROR: PTH Data failed checksum, check sensor!")
      gcs:send_text(0, "Incoming string: " .. data .. string.format(" size: %d", #data))
    end
  end

  return update, 250 -- reschedules the loop every 250ms
end


return update() -- run immediately before starting to reschedule
