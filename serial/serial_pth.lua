--[[
PTH (Samamet) Serial Sensor Reading

Read the data from the serial line that the PTH is connected to, then decode the
messages from it, and log the data to the autopilots BIN file.

Author: Justin Tussey
Last Updated: 2024-06-17
]]--

-- Global Constants

-- initialize serial connection
local BAUD_RATE = 9600

-- Max length of the messages from the anemometer
local MAX_MESSAGE_LENGTH = 60

-- Return rate and calculations
local SCHEDULE_RATE = 100 --milliseconds
local TIME_BETWEEN_DATA = 1000 --milliseconds
-- number of how many loops we need for us to properly flag that the sensor is
-- not sending data (// is floor division (removes decimal))
local LOOPS_TO_FAIL = (TIME_BETWEEN_DATA // SCHEDULE_RATE) + (1)

-- error type table
local ERROR_LIST = {
  "No data received",      -- 1
  "Checksum fail",         -- 2
  "Data parsing fail",     -- 3
  "Invalid data frame"     -- 4
}

-- find the serial first (0) scripting serial port instance
-- SERIALx_PROTOCOL 28
local PORT = assert(serial:find_serial(0),"Could not find Scripting Serial Port")

-- begin the serial port
PORT:begin(BAUD_RATE)
PORT:set_flow_control(0)

-- Global Variables
-- variable to count iterations without recieving message from the Samamet
local loops_since_data_received = 0


-- Take in string and verify that it is a valid message frame. Specifically a
-- message in the NMEA-0183 format, which starts with "$" and ends with <CR><LF>
-- (DOS line ending)
---@param message_string string
---@return boolean
function verify_valid_frame(message_string)
  -- get the first character in the message, which should be "$" if valid
  local head = string.sub(message_string, 1, 1)

  -- get the last two characters in the message, which should be <CR><LF>
  -- ("\r\n") if valid
  local tail = string.sub(message_string, #message_string-1, #message_string)

  if (head ~= "$") then
    return false
  elseif (tail ~= "\r\n") then
    return false
  end

  return true
end

-- Calculate checksum by taking sub-string from $ to * (but not including
-- them), then XORing each of the ASCII characters in the string with the next
-- one in the string. Then compare the result in hex with the 2 character hex
-- code after the *. Returns a boolean value of whether it passed the check.
---@param  message_string string
---@return boolean
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

  -- Take the string of the checksum, convert it to a number, specifing that its
  -- a hexadecimal number
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

-- Take the data from a verified message string and split up the string, using
-- a comma as a delimiter. Then take the measurements from the data and log them
-- to the BIN file. Return whether the parsing failed or passed
---@param  message_string string
---@return boolean
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
  gcs:send_text(7, "p:"  .. string.format(" %.1f  ", measurements_table[1])  ..
                   "t1:" .. string.format(" %.1f  ", measurements_table[2])  ..
                   "t2:" .. string.format(" %.1f  ", measurements_table[3])  ..
                   "h:"  .. string.format(" %.1f  ", measurements_table[4])  ..
                   "t3:" .. string.format(" %.1f", measurements_table[5])
  )

  -- return whether data input data matched needed format (table with 5
  -- elements)
  return log_data(measurements_table)
end

-- take the input table, and check if it has 5 elements in it, then write that
-- data to the BIN file, with the appropriate names and units
---@param measurements_table table
---@return boolean
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
               'NNNNNN',                                   -- data type (char[16])
               'POO%O-',                                   -- units,(Pa, C, C, % (humidity), C)
               '------',                                   -- multipliers (- signifies no multiplier)
               measurements_table[1],                      -- data for labels
               measurements_table[2],
               measurements_table[3],
               measurements_table[4],
               measurements_table[5],
               "Normal")
  return true
end

-- called when an error is detected, and writes all zeros to the BIN file and
-- writes what kind of error was experienced
---@param error_type string
function log_error(error_type)
  logger:write('SAMA', 'pres,temp1,temp2,hum,temp3,error', -- section name and labels
               'NNNNNN',                                   -- data type (char[16])
               'POO%O-',                                   -- units,(Pa, C, C, % (humidity), C)
               '------',                                   -- multipliers (- signifies no multiplier)
               '0', '0', '0', '0', '0', error_type)        -- zeros for labels since there is an error with the PTH or its data
end

-- primary loop. Gets message from serial interface, verifies that it is valid,
-- then logs the data to the autopilots BIN file
---@return function
---@return integer
function update()
  local n_bytes = PORT:available()

  -- Check if there are any bytes available, if not log an error after the
  -- specified number of failed loops
  if n_bytes <= 0 then
    loops_since_data_received = loops_since_data_received + 1
    if loops_since_data_received >= LOOPS_TO_FAIL then
      log_error(ERROR_LIST[1])
      gcs:send_text(0, "ERROR: Disconnected Sensor")
    end
    return update, SCHEDULE_RATE
  end

  -- get MAX_MESSAGE_LENGTH bytes from the serial line, and place them into a
  -- string
  local message_string = PORT:readstring(MAX_MESSAGE_LENGTH)

  -- make sure we got a message from the serial line
  if (message_string == nil or #message_string <= 0) then
    return update, SCHEDULE_RATE
  end

  -- clear loops since data received since we passed the checks to verify we got
  -- a message
  loops_since_data_received = 0

  -- check if we have a valid data frame, which checks for the NMEA-0183
  -- sentence start and ending characters. If not valid (which means we do not
  -- have a complete message) clear the queue and return
  if not (verify_valid_frame(message_string)) then
    -- Read the available bytes in the queue and do nothing with them.
    -- We effectively clear the queue
    PORT:readstring(PORT:available():toint())
    log_error(ERROR_LIST[4])
    return update, SCHEDULE_RATE
  end

  -- verify the checksum in the message to check if the data has been corrupted
  -- log an error if we fail the check
  if not (verify_checksum(message_string)) then
    log_error(ERROR_LIST[2])
    gcs:send_text(0, "ERROR: Data failed checksum")
    gcs:send_text(7, message_string)
    return update, SCHEDULE_RATE
  end

  -- parse and log the data, log error if we experience an issue. Most likely
  -- means that the data was corrupted and was not caught by the checksum
  -- verification (unlikely)
  if not (parse_data(message_string)) then
    log_error(ERROR_LIST[3])
    gcs:send_text(0, "ERROR: Failed to parse data")
    return update, SCHEDULE_RATE
  end

  -- gcs:send_text(7, message_string)
  -- reschedule the update function to run every SCHEDULE_RATE milliseconds
  return update, SCHEDULE_RATE
end

-- run immediately before starting to reschedule
return update()
