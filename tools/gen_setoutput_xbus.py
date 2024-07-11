# The original hex string
# FA FF C0 18 10 60 FF FF 20 30 00 64 40 20 00 64 80 20 00 64 C0 20 00 64 E0 20 FF FF FD
# copy from the 5th byte to the last byte without checksum byte
hex_string = "10 60 FF FF 40 20 00 64 80 20 00 64 C0 20 00 64 20 30 00 64 50 42 00 64 50 22 00 64 D0 12 00 64"
# FA FF C0 10 10 60 FF FF 40 20 00 64 80 20 00 64 C0 20 00 64 B7
# FA FF C0 18 10 60 FF FF 20 10 00 64 40 20 00 64 80 20 00 64 C0 20 00 64 E0 20 FF FF 1D
# FA FF C0 1C 10 60 FF FF 20 10 00 64 40 20 00 64 80 20 00 64 C0 20 00 64 40 30 00 64 E0 20 FF FF 45
#FA FF C0 20 10 60 FF FF 40 20 00 64 80 20 00 64 C0 20 00 64 20 30 00 64 50 42 00 64 50 22 00 64 D0 12 00 64 E1



# Split the string into a list of hex values
hex_values = hex_string.split()

# Format each hex value with '0x' prefix and convert to uppercase
formatted_hex_values = [f"0x{value}" for value in hex_values]

# Join the list into a single string with ', ' as separator
result = ", ".join(formatted_hex_values)

# Print the result
print(result)

##with this result, copy paste to the application.cpp line 193, assign to the output_config_payload.