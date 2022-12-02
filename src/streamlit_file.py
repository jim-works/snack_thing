import socket
import os.path
from PIL import Image
import streamlit as st

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65423  # The port used by the server


def coor_for_robot(x, y):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        st.write("Moving!")
        s.sendall(bytes(f"{x},{y}", "utf-8"))
        data = s.recv(1024)
        st.write("Done!")


def submit_coordinates(x, y, option):
    coord = ()
    if (option == "Pretzels"):
        coord = (3.05, -4)
    elif (option == "Gold Fish"):
        coord = (3.05, 0)
    else:
        coord = (3.05, 4)
    st.write(coord)
    coor_for_robot(coord[0], coord[1])
    coor_for_robot(x, y)
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
