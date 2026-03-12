import os 
from dotenv import load_dotenv

load_dotenv()

SERVER_HOST = os.getenv('SERVER_HOST_IP_ADDRESS')
SERIAL_PORT = os.getenv('LOCAL_SERIAL_PORT')

print(SERVER_HOST)
print(SERIAL_PORT)