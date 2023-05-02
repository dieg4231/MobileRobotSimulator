import time
import tkinter as tk
from tkinter import filedialog

time.sleep(0.1)
window = tk.Tk(className='Robot scan')
window.geometry("200x100")
label = tk.Label(text = "Please wait...", font='Arial 17 bold')
label.place(relx=0.5, rely=0.5)
label.pack(pady=30)

window.after(2600, lambda:window.destroy())
window.mainloop()

