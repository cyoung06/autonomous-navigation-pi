import os

os.mkdir(f'dataset/')
with open('processed_wifi_data.csv', 'r') as f2:
    lines = f2.read().splitlines()
    for line in lines:
        stuffs = [a.strip() for a in line.split(",")]

        if not os.path.exists(f'dataset/{stuffs[1]}p{stuffs[2]}'):
            os.mkdir(f'dataset/{stuffs[1]}p{stuffs[2]}')

        os.rename(f'images/{stuffs[0]}.jpg', f'dataset/{stuffs[1]}p{stuffs[2]}/{stuffs[0]}.jpg')