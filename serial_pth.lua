--[[
PTH Serial Sensor Reading

Read the data from the serial line that the PTH is connected to, then decode the
messages from it, and log the data to the autopilots BIN file.

Author: Justin Tussey
Last Updated: 2024-05-14
]]--


-- Calculate checksum by taking string from '$' to '*' (but not including them),
-- then XORing each of the ASCII characters in the string with the next one in
-- the string. Then compare the result in hex with the 2 character hex code
-- after the "*"
function verify_checksum(message_string)
  -- starting value of zeros, which will not effect the first XOR
  local checksum = 0x0

  -- Take the message string and convert it to bytes, then place into curly
  -- braces to convert it to an array of bytes, where each byte is an ascii
  -- character. This allows us to iterate over it and do bitwise operations on
  -- it (lua does not support bitwise ops on strings)
  local string_bytes = {message_string:byte(1, #message_string)}

  -- Iterate over the character array, and XOR each character with the
  -- previous "summed" XOR operations
  for i=1, #string_bytes do
    checksum = (checksum ~ string_bytes[i])
  end

  return checksum
end

a = "UKPTH,000E,098152.5,Pa,23.17,C,22.90,C,42.21,%,22.45,C"
x = verify_checksum(a)

print(string.format("%X", x))


-- function update()

--   verify_checksum("UKPTH,000E,098152.5,Pa,23.17,C,22.90,C,42.21,%,22.45,C")

--   return update, 1000 -- reschedules the loop every 1000ms (temporary)
-- end

-- return update() -- run immediately before starting to reschedule
