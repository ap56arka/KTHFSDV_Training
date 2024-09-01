import subprocess
import sys

def install(package):
    subprocess.check_call([sys.executable, "-m", "pip", "install", package])

# List your packages here
packages = ["numpy", "matplotlib", "customtkinter"]

for package in packages:
    install(package)


print("----------All required packages installed!----------")