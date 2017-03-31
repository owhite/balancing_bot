#!/usr/bin/env python

import time
import os
import serial
from decimal import Decimal
from Tkinter import *

class layout:
   def __init__(self, master):
      self.port = '/dev/cu.usbmodem1754161'
      self.serial_exists = 0

      try:
         s = serial.Serial(self.port, 38400)
         time.sleep(2) 
         s.flushInput()  # flush input buffer
         s.flushOutput()
         cmd = 'probe_device\n'
         s.write(cmd)
         time.sleep(1) 
         msg = s.read(s.inWaiting())
         if (msg.rstrip() == 'balancing'):
            self.serial_exists = 1
            self.serial = s
         else:
            s.close()

      except (OSError, serial.SerialException):
         print "dintna find serial"

      row = 0;
      self.master = master
      self.frame = Frame(self.master)

      row += 1
      if self.serial_exists:
         Label(self.frame, text="Balance Bot Control").grid(row=row, columnspan=3)
      else:
         Label(self.frame, text="Balance Bot: NOT FOUND").grid(row=row, columnspan=3)

      row += 1

      self.kPvar = StringVar()
      self.kPvar.set('0.0')
      Label(self.frame, textvariable=self.kPvar).grid(row=row, column = 0, sticky=W, padx=4)
      self.e1 = Entry(self.frame)
      self.e1.grid(row=row, column = 1, sticky=W, padx=4)
      row += 1

      self.kIvar = StringVar()
      Label(self.frame, textvariable=self.kIvar).grid(row=row, column = 0, sticky=W, padx=4)
      self.e2 = Entry(self.frame)
      self.e2.grid(row=row, column = 1, sticky=W, padx=4)
      row += 1

      self.kDvar = StringVar()
      Label(self.frame, textvariable=self.kDvar).grid(row=row, column = 0, sticky=W, padx=4, pady=4)
      self.e3 = Entry(self.frame)
      self.e3.grid(row=row, column = 1, sticky=W, padx=4, pady=4)
      row += 1

      self.RXvar = StringVar()
      self.RXvar.set('RXed')
      self.RXlabel = Label(self.frame, textvariable=self.RXvar).grid(row=row, columnspan = 2, sticky=W, padx=4, pady=4)
      row += 1

      Button(self.frame, text='Send',  command=self.send_PIDs).grid(row=row,column=0)
      Button(self.frame, text='Pause', command=self.pause_bot).grid(row=row,column=1)
      row += 1

      Button(self.frame, text='Write',  command=self.write_eeprom).grid(row=row,column=0)
      Button(self.frame, text='Zero',  command=self.zero_bot).grid(row=row,column=1)
      row += 1

      Button(self.frame, text='Quit',  command=master.quit).grid(row=row,sticky=W)

      self.update_PIDs()

      self.frame.pack()

   def update_PIDs(self):
      result = self.send_command('get_PIDs\n')
      l = result.split(' ')
      self.kPvar.set(l[0])
      self.kIvar.set(l[1])
      self.kDvar.set(l[2])


   def send_PIDs(self):
      if (len(self.e1.get()) > 0):
         cmd = 'P %s\n' % self.e1.get()
         self.send_command(cmd)
      if (len(self.e2.get()) > 0):
         cmd = 'I %s\n' % self.e2.get()
         self.send_command(cmd)
      if (len(self.e3.get()) > 0):
         cmd = 'D %s\n' % self.e3.get()
         self.send_command(cmd)
      self.update_PIDs()


   def pause_bot(self):
      self.send_command('p\n');

   def write_eeprom(self):
      self.send_command('write\n');
      self.update_PIDs()

   def zero_bot(self):
      print "zeroing"

   def send_command(self, cmd):
      self.RXvar.set(cmd)
      if (self.serial_exists):
         self.serial.flushInput()  # flush input buffer
         self.serial.flushOutput() # flush output buffer
         self.serial.write(cmd)
         time.sleep(1)
         msg = self.serial.read(self.serial.inWaiting())
         self.RXvar.set(msg.rstrip())
         self.serial.flushOutput() # flush output buffer

      else:
         self.RXvar.set("no serial")
      return msg.strip()
      
   def format_float(self, f):
      d = Decimal(str(f));
      return d.quantize(Decimal(1)) if d == d.to_integral() else d.normalize()


def main(): 
    root = Tk()
    app = layout(root)
    root.mainloop()

if __name__ == '__main__':
    main()


