# Installation of python3 requirements
If both python 2 and python 3 are installed on your machine, use the following:

>pip3 install --user  -r "requirements.txt"

If you have only python3 installed on your machine, use the following:

>pip install --user  -r "requirements.txt"

# Requirement for Tkinter based visualization
Some samples are dependent on Tkintker and PIL. You can install them on debian systems with the following commands:
>sudo apt install python3-pil.imagetk
