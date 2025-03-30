import websocket

# Connect to WebSocket server
ws = websocket.WebSocket()
ws.connect("ws://172.30.1.93")
print("Connected to WebSocket server")

while True:
    # Ask the user for some input and transmit it
    str = input("Say something: ")
    ws.send(str)

    # Wait for server to respond and print it
    result = ws.recv()
    print("Received: " + result)
    
    if str == 's':
        break

# Gracefully close WebSocket connection
ws.close()
