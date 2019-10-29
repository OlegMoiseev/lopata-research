import json

path_settings = 'settings.json'
with open(path_settings) as settings_file:
    settings = json.load(settings_file)
    parameters = ['1', '5', '8']

    for i in range(len(settings[parameters[0]])):
        for j in range(len(settings[parameters[1]])):
            for k in range(len(settings[parameters[2]])):
                print(settings[parameters[0]][i], settings[parameters[1]][j], settings[parameters[2]][k])
