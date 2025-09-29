#!/bin/python3
import re
import os
import shutil
import sys

# TODO: Fix drivetrain_generated_text to be whatever pheonix tuner produces
DRIVETRAIN_GENERATED_TEXT = 'drivetrain'
REPLACE_TEXT = 'Drivetrain'
CONSTANTS_PATH = './src/main/java/frc/robot/generated/TunerConstants.java'
PHOENIX_VERSION = '25.3.1'

ERROR_TEXT = """
This script was made to parse TunerConstants generated uner phoenix6 {}.
You may backup the old constants and verify the new one.
If it compiles and drives okay, update the PHOENIX_VERSION constant. Otherwise the script will need to be changed.
If you choose not to backup the old constants, the script will terminate.
"""

USAGE_TEXT = f"""
{sys.argv[0]} [-r]
-r: restore from backup
"""


def check_phoenix():
    if not os.path.exists(f'./vendordeps/Phoenix6-{PHOENIX_VERSION}.json'):
        print(ERROR_TEXT.format(PHOENIX_VERSION))
        response = input('Generate backup? [Y/n] ').strip().lower()
        if response == 'y':
            return True
        elif response == 'n':
            print('No modifications were made to the project.\nDone')
            exit(0)
        else:
            print("Invalid option. Please type 'y' or 'n' (case insensitive)")
            exit(1)
    return False # The excpected version was found

def main():
    print('Checking phoenix6 version ...')
    backup = check_phoenix()

    print('Checking if TunerConstants.java exists')
    if not os.path.exists(CONSTANTS_PATH):
        print('Tuner constants not found at excpected location')
        exit(1)

    print('Parsing ...')
    f = open("./src/main/java/frc/robot/generated/TunerConstants.java", 'r')
    tunerconstants = f.readlines()
    f.close()
    tunerconstants = {num: line for num,line in enumerate(tunerconstants)}

    to_be_renamed_has_dt = {num: line for num, line in tunerconstants.items() if DRIVETRAIN_GENERATED_TEXT in line.lower()}
    to_be_renamed_wholeword = {num: (line, re.search(r"\b" + re.escape(DRIVETRAIN_GENERATED_TEXT) + r"\b", line.lower())) for num, line in to_be_renamed_has_dt.items()}
    to_be_renamed_wholeword = {num: line for num, line in to_be_renamed_wholeword.items() if line[1]}

    renamed = {}
    for num, line in to_be_renamed_wholeword.items():
        if len(line[0]) >= 2:
            if line[0][0] == '*' or line[0][0:2] == '//':
                continue
            if line[0][line[1].span()[0]-1] == '"':
                continue
        span = line[1].span()
        first = line[0][0:span[0]]
        second = line[0][span[1]:len(line[0])]
        renamed[num] = first + REPLACE_TEXT + second

    tunerconstants.update(renamed)
    tunerconstants_list = list(tunerconstants.values())

    print('Backing up ...')
    if backup:
        shutil.copyfile(CONSTANTS_PATH, CONSTANTS_PATH+'backup')
    
    print('Writing file ...')
    os.remove(CONSTANTS_PATH)
    output = open(CONSTANTS_PATH, 'x')
    output.write(''.join(tunerconstants_list))
    output.close()

    print('Done')

def restore():
    print('Restoring backup ...')
    os.remove(CONSTANTS_PATH)
    shutil.copyfile(CONSTANTS_PATH+'backup', CONSTANTS_PATH)
    print('Done.')

def usage():
    print(USAGE_TEXT)
    exit(1)

if __name__ == "__main__":
    if len(sys.argv) > 2:
        exit(1)
    elif len(sys.argv) == 2:
        restore()
    else:
        main()
