import os
import sys
os.chdir("D:\Dymanic_simulation_BSRM")

def delete_file():
    file_delete_list=["info.exe","info.dat","info.txt","title.txt"]
    for file_name in file_delete_list:
        try:
            os.remove(file_name)
        except OSError:
            pass


if __name__ == '__main__':  
    delete_file() 
    os.startfile("execut_command.bat")

