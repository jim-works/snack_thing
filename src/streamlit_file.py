import socket
import os.path
from PIL import Image
import streamlit as st

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65422  # The port used by the server


def send_command(dispenser_id, x, y):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        st.write("Moving!")
        s.sendall(bytes(f"{dispenser_id},{x},{y}", "utf-8"))
        data = s.recv(1024)
        st.write("Done!")

def submit_coordinates(x, y, option):
    dispensers = {
        "Pretzels": 0,
        "Gold Fish": 1,
        "Cheese Its": 2
    }
    send_command(dispensers[option], x, y)
    return


st.header("Snack Helper \n")

st.subheader("Insert the coordinates you would like the robot to travel to")
x_number = st.number_input('x coordinate')
y_number = st.number_input('y coordinate')
option = st.selectbox(
    'Which Snack would you like?',
    ('Pretzels', 'Gold Fish', 'Cheese Its'))

if st.button('Submit Coordinates'):
    submit_coordinates(x_number, y_number, option)
st.subheader("\n Here is your house!")
st.write("The x axis is shown from down to up (red line)")
st.write("The y axis is shown from left to right (green line)")
st.write("Coordinate (0,0) is located where the green box is")
image = Image.open('plane.png')

st.image(image, caption='coordinate frame')
