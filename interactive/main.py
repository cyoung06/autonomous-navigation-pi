from tkinter import *

root = Tk()
root.geometry('1000x1000')


from peripherals.wifi import get_nearby_routers


def task():
    print("hello")
    root.after(2000, task)  # reschedule event in 2 seconds

root.after(2000, task)
root.mainloop()