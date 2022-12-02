import os.path
from PIL import Image
import streamlit as st


def submit_coordinates(x, y):
    #st.write(x, y)
    return


st.header("Snack Helper \n")

st.subheader("Insert the coordinates you would like the robot to travel to")
x_number = st.number_input('x coordinate')
y_number = st.number_input('y coordinate')
if st.button('Submit Coordinates'):
    submit_coordinates(x_number, y_number)
st.subheader("\n Here is your house!")
st.write("The x axis is shown from left to right")
st.write("The y axis is shown from down to up")
st.write("Coordinate (0,0) is located where the green box is")
image = Image.open('plane.png')

st.image(image, caption='coordinate frame')
