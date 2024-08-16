file_path = '/Users/barbaracristina/Documents/GitHub/IC/rotagurobi.txt'

# Read the file
with open(file_path, 'r') as file:
    content = file.read()

# Remove commas
content = content.replace(',', '')

# Write the modified content back to the file
with open(file_path, 'w') as file:
    file.write(content)