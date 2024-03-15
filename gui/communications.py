from cProfile import label
from tkinter import *
import serial
import threading
import serial.tools.list_ports
import ssl
import paho.mqtt.client as mqtt
from PIL import ImageTk, Image


root = Tk()
root.title("COMMS")
root.geometry("1000x415")
root.configure(background="black")

# Main Class For GUI
class Elder:
    # initalisation function
    def __init__(self, master):
        self.minimum_value = 1000
        self.maximum_value = -1000
        self.cont_value = 0
        self.tare_on = False
        self.current_tare = 0
        self.wifi_mode = False
        self.connected = False
        self.current_mass_int = 0
        self.wifi_on_img = ImageTk.PhotoImage((Image.open("wifi.jpg")).resize((20, 20)))
        self.wifi_off_img = ImageTk.PhotoImage(
            (Image.open("wifioff.jpg")).resize((20, 20))
        )

        self.open = False
        self.t2 = threading.Thread(target=self.thread_open_serial)
        self.t2.start()

        self.display_frame = Frame(
            master,
            bg="gray50",
            width=950,
            height=150,
        )
        self.display_frame_upper_label = Frame(
            self.display_frame,
            bg="gray35",
            width=950,
            height=13,
        )
        self.display_frame_upper = Frame(
            self.display_frame,
            bg="gray35",
            width=950,
            height=50,
        )
        self.display_frame_lower_label = Frame(
            self.display_frame,
            bg="gray50",
            width=950,
            height=13,
        )
        self.display_frame_lower = Frame(
            self.display_frame,
            bg="gray50",
            width=950,
            height=100,
        )

        self.weighframe_label = Frame(master, bg="gray76", width=950, height=13)
        self.weighframe = Frame(master, bg="gray76", width=950, height=50)
        self.countingframe_label = Frame(master, bg="gray76", width=950, height=13)
        self.countingframe = Frame(master, bg="gray76", width=950, height=50)
        self.passfailframe_label = Frame(master, bg="gray76", width=950, height=13)
        self.passfailframe = Frame(master, bg="gray76", width=950, height=50)

        self.display_frame.pack(padx=10, pady=(10, 10))
        self.display_frame_upper_label.pack()
        self.display_frame_upper.pack()
        self.display_frame_lower_label.pack()
        self.display_frame_lower.pack()
        self.weighframe_label.pack()
        self.weighframe.pack(padx=10, pady=(0, 10))
        self.countingframe_label.pack()
        self.countingframe.pack(padx=10, pady=(0, 10))
        self.passfailframe_label.pack()
        self.passfailframe.pack(padx=10, pady=(0, 10))
        self.display_frame.pack_propagate(0)
        self.display_frame_upper_label.pack_propagate(0)
        self.display_frame_upper.pack_propagate(0)
        self.display_frame_lower_label.pack_propagate(0)
        self.display_frame_lower.pack_propagate(0)
        self.weighframe_label.pack_propagate(0)
        self.weighframe.pack_propagate(0)
        self.countingframe_label.pack_propagate(0)
        self.countingframe.pack_propagate(0)
        self.passfailframe_label.pack_propagate(0)
        self.passfailframe.pack_propagate(0)

        # LABELS
        Label(
            self.display_frame_upper_label,
            text="Zero Value",
            width=20,
            font=("arial", 10, "bold"),
            padx=30,
            bg="gray35",
        ).pack(side=LEFT)

        Label(
            self.display_frame_upper_label,
            text="Mode",
            width=25,
            font=("arial", 10, "bold"),
            padx=10,
            bg="gray35",
        ).pack(side=LEFT)

        Label(
            self.display_frame_upper_label,
            text="Calibrate",
            width=15,
            font=("arial", 10, "bold"),
            padx=10,
            bg="gray35",
        ).pack(side=LEFT)

        Label(
            self.display_frame_upper_label,
            text="Brightness -",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray35",
        ).pack(side=LEFT)
        Label(
            self.display_frame_upper_label,
            text="Brightness +",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray35",
        ).pack(side=LEFT)
        Label(
            self.display_frame_upper_label,
            text="Wifi",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray35",
        ).pack(side=LEFT)
        Label(
            self.display_frame_upper_label,
            text="         ",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray35",
        ).pack(side=LEFT)

        # Display frame Lower
        Label(
            self.display_frame_lower_label,
            text="Mass Value",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray50",
        ).pack(side=LEFT, expand=True)
        Label(
            self.display_frame_lower_label,
            text="ADC Value",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray50",
        ).pack(side=LEFT, expand=True)

        # Weigh Frame
        Label(
            self.weighframe_label,
            text="Select Mode",
            width=10,
            font=("arial", 10, "bold"),
            padx=30,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        Label(
            self.weighframe_label,
            text="Tare On",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        Label(
            self.weighframe_label,
            text="Tare Off",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        Label(
            self.weighframe_label,
            text="Tare Value",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        Label(
            self.countingframe_label,
            text="Select Mode",
            width=10,
            font=("arial", 10, "bold"),
            padx=30,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        Label(
            self.countingframe_label,
            text="Reference Count",
            width=13,
            font=("arial", 10, "bold"),
            padx=3,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        Label(
            self.countingframe_label,
            text="Set Count",
            width=10,
            font=("arial", 10, "bold"),
            padx=3,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        Label(
            self.countingframe_label,
            text="Count Value",
            width=9,
            font=("arial", 10, "bold"),
            padx=3,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        Label(
            self.countingframe_label,
            text="Reference mass",
            width=13,
            font=("arial", 10, "bold"),
            padx=3,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        Label(
            self.countingframe_label,
            text="Unit Mass",
            width=8,
            font=("arial", 10, "bold"),
            padx=3,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        Label(
            self.passfailframe_label,
            text="Select Mode",
            width=20,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray76",
        ).pack(side=LEFT, expand=True)
        Label(
            self.passfailframe_label,
            text="Lower Lim",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray76",
        ).pack(side=LEFT, expand=True)
        Label(
            self.passfailframe_label,
            text="Upper Lim",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray76",
        ).pack(side=LEFT, expand=True)
        Label(
            self.passfailframe_label,
            text="Set Pass/Fail",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray76",
        ).pack(side=LEFT, expand=True)
        Label(
            self.passfailframe_label,
            text="Pass/Fail",
            width=10,
            font=("arial", 10, "bold"),
            padx=5,
            bg="gray76",
        ).pack(side=LEFT, expand=True)

        # Current modes
        self.current_mode = StringVar(self.display_frame_upper)
        self.current_mode.set("Weigh Mode")

        self.wifi_img = Label(self.display_frame_upper, image=self.wifi_off_img)
        self.wifi_img.pack(side=RIGHT, padx=10, pady=10, expand=True)

        # Wifi Mode
        self.wifi = Button(self.display_frame_upper)
        self.wifi.config(
            width=7,
            font=("arial", 10, "bold"),
            text="WIFI ON",
            bg="red",
            command=self.wifi_on,
        )
        self.wifi.pack(side=RIGHT, padx=10, pady=10, expand=True)

        # increase brightness
        self.bright_up = Button(self.display_frame_upper)
        self.bright_up.config(
            width=4, font=("arial", 10, "bold"), text="+", command=self.increase_bright
        )
        self.bright_up.pack(side=RIGHT, padx=10, pady=10, expand=True)
        # increase brightness
        self.bright_down = Button(self.display_frame_upper)
        self.bright_down.config(
            width=4, font=("arial", 10, "bold"), text="-", command=self.decrease_bright
        )
        self.bright_down.pack(side=RIGHT, padx=10, pady=10, expand=True)

        # Calibrate
        self.calibrate = Button(self.display_frame_upper)
        self.calibrate.config(
            width=9,
            font=("arial", 10, "bold"),
            text="CALIBRATE",
            command=self.open_popup,
        )
        self.calibrate.pack(side=RIGHT, padx=10, pady=10, expand=True)
        # Current reading
        self.current_reading = StringVar(self.display_frame_lower)

        # Raw Reading
        self.raw_reading = StringVar(self.display_frame_lower)

        self.count_set_value = StringVar(self.weighframe)
        self.count_set_value.set("COUNT")

        self.lower_value = StringVar(self.passfailframe)
        self.lower_value.set("LOWER")

        self.upper_value = StringVar(self.passfailframe)
        self.upper_value.set("UPPER")

        self.tare_value = StringVar(self.weighframe)
        self.tare_value.set(0)

        self.count_value = StringVar(self.weighframe)
        self.count_value.set(0)

        self.ref_mass = StringVar(self.weighframe)
        self.ref_mass.set(0)

        self.unit_mass = StringVar(self.weighframe)
        self.unit_mass.set(0)

        self.pass_value = StringVar(self.weighframe)
        self.pass_value.set("--")

        self.zero_weight = StringVar(self.display_frame_upper)
        self.zero_weight.set("Zero:-")

        # SET THE DISPLAY

        # Current zero
        self.zero_display = Label(
            self.display_frame_upper, textvariable=self.zero_weight, bg="gray35"
        )
        self.zero_display.config(width=15, font=("arial", 15, "bold"))
        self.zero_display.pack(side=LEFT, padx=10, pady=10, expand=True)

        # Current Mode
        self.display_mode = Label(
            self.display_frame_upper, textvariable=self.current_mode, bg="gray35"
        )
        self.display_mode.config(width=15, font=("arial", 15, "bold"))
        self.display_mode.pack(side=LEFT, padx=10, pady=10, expand=True)

        # Bottom Left
        self.display_raw = Label(
            self.display_frame_lower, textvariable=self.raw_reading, bg="gray50"
        )
        self.display_raw.config(width=7, font=("arial", 25, "bold"))
        self.display_raw.pack(side=LEFT, padx=20, pady=10, expand=True)

        # curent reading
        self.display_current_reading = Label(
            self.display_frame_lower, textvariable=self.current_reading, bg="gray50"
        )
        self.display_current_reading.config(width=7, font=("arial", 25, "bold"))
        self.display_current_reading.pack(side=RIGHT, padx=20, pady=10, expand=True)

        # buttons
        self.button_select_weigh = Button(self.weighframe)
        self.button_select_weigh.config(
            width=16,
            font=("arial", 15, "bold"),
            text="Select Weigh Mode",
            command=self.weigh_mode,
        )
        self.button_select_weigh.pack(side=LEFT, padx=10, pady=10, expand=True)
        self.button_tare = Button(self.weighframe)
        self.button_tare.config(
            width=7,
            font=("arial", 15, "bold"),
            text="TARE",
            command=self.tare,
            bg="red",
        )
        self.button_tare.pack(side=LEFT, padx=10, pady=10, expand=True)

        self.button_tare_off = Button(self.weighframe)
        self.button_tare_off.config(
            width=9,
            font=("arial", 15, "bold"),
            text="TARE OFF",
            command=self.tare_off,
            bg="green",
        )
        self.button_tare_off.pack(side=LEFT, padx=10, pady=10, expand=True)

        # curent tare
        self.display_tare = Label(
            self.weighframe, textvariable=self.tare_value, bg="gray76"
        )
        self.display_tare.config(width=7, font=("arial", 15, "bold"))
        self.display_tare.pack(side=LEFT, padx=10, pady=10, expand=True)

        # slect count mode
        self.button_select_count = Button(self.countingframe)
        self.button_select_count.config(
            width=16,
            font=("arial", 15, "bold"),
            text="Select Count Mode",
            command=self.counting_mode,
        )
        self.button_select_count.pack(side=LEFT, padx=10, pady=10, expand=True)

        self.button_enter_count = Entry(self.countingframe)
        self.button_enter_count.config(
            width=7,
            font=("arial", 15, "bold"),
            textvariable=self.count_set_value,
        )
        self.button_enter_count.pack(side=LEFT, padx=10, pady=10, expand=True)

        self.button_set_counting = Button(self.countingframe)
        self.button_set_counting.config(
            width=7,
            font=("arial", 15, "bold"),
            text="SET",
            command=self.set_count,
        )
        self.button_set_counting.pack(side=LEFT, padx=10, pady=10, expand=True)

        # curent tare
        self.display_count = Label(
            self.countingframe, textvariable=self.count_value, bg="gray76"
        )
        self.display_count.config(width=7, font=("arial", 15, "bold"))
        self.display_count.pack(side=LEFT, padx=10, pady=10, expand=True)

        self.display_ref_mass = Label(
            self.countingframe, textvariable=self.ref_mass, bg="gray76"
        )
        self.display_ref_mass.config(width=7, font=("arial", 15, "bold"))
        self.display_ref_mass.pack(side=LEFT, padx=10, pady=10, expand=True)

        self.display_unit_mass = Label(
            self.countingframe, textvariable=self.unit_mass, bg="gray76"
        )
        self.display_unit_mass.config(width=7, font=("arial", 15, "bold"))
        self.display_unit_mass.pack(side=LEFT, padx=10, pady=10, expand=True)

        self.button_select_pass_fail = Button(self.passfailframe)
        self.button_select_pass_fail.config(
            width=18,
            font=("arial", 15, "bold"),
            text="Select P/F Mode",
            command=self.pass_fail_mode,
        )
        self.button_select_pass_fail.pack(side=LEFT, padx=10, pady=10, expand=True)

        self.button_enter_lower = Entry(self.passfailframe)
        self.button_enter_lower.config(
            width=7,
            font=("arial", 15, "bold"),
            textvariable=self.lower_value,
        )
        self.button_enter_lower.pack(side=LEFT, padx=10, pady=10, expand=True)

        self.button_enter_upper = Entry(self.passfailframe)
        self.button_enter_upper.config(
            width=7,
            font=("arial", 15, "bold"),
            textvariable=self.upper_value,
        )
        self.button_enter_upper.pack(side=LEFT, padx=10, pady=10, expand=True)

        self.button_set_pass_fail = Button(self.passfailframe)
        self.button_set_pass_fail.config(
            width=7,
            font=("arial", 15, "bold"),
            text="SET",
            command=self.set_pass_fail,
        )
        self.button_set_pass_fail.pack(side=LEFT, padx=10, pady=10, expand=True)

        # pass fail
        self.display_pass_fail = Label(
            self.passfailframe, textvariable=self.pass_value, bg="gray76"
        )
        self.display_pass_fail.config(width=7, font=("arial", 20, "bold"))
        self.display_pass_fail.pack(side=LEFT, padx=10, pady=10, expand=True)

        self.run()

    # UTIL FUNCTIONS
    # wifi function
    def wifi_on(self):
        if self.wifi_mode == False:
            self.client.publish("team26/uplink", ("PO:" + "1"))
            self.wifi_mode = True
            self.wifi.config(bg="green")
            self.wifi_img.config(image=self.wifi_on_img)
            self.ser.write(("CM:" + "1" + "\n").encode("utf-8"))

        elif self.wifi_mode == True:
            self.client.publish("team26/uplink", ("PO:" + "0"))
            self.wifi_mode = False
            self.wifi.config(bg="red")
            self.wifi_img.config(image=self.wifi_off_img)
            self.client.publish("team26/uplink", ("CM:" + "0"))

    # weight mode function
    def weigh_mode(self):
        if self.wifi_mode == True:
            self.client.publish("team26/uplink", ("MD:" + "0"))
        else:
            self.ser.write(("MD:" + "0" + "\n").encode("utf-8"))

    # counting mode function
    def counting_mode(self):
        if self.wifi_mode == True:
            self.client.publish("team26/uplink", ("MD:" + "4"))
        else:
            self.ser.write(("MD:" + "4" + "\n").encode("utf-8"))

    # pass fail mode function
    def pass_fail_mode(self):
        if self.wifi_mode == True:
            self.client.publish("team26/uplink", ("MD:" + "2"))
        else:
            self.ser.write(("MD:" + "2" + "\n").encode("utf-8"))

    # tare function
    def tare(self):
        if self.wifi_mode == True:
            self.client.publish("team26/uplink", ("TA:" + "1"))
        else:
            self.ser.write(("TA:" + "1" + "\n").encode("utf-8"))

    # tare off function
    def tare_off(self):
        if self.wifi_mode == True:
            self.client.publish("team26/uplink", ("TA:" + "0"))
        else:
            self.ser.write(("TA:" + "0" + "\n").encode("utf-8"))

    # set count function
    def set_count(self):
        if self.wifi_mode == True:
            print(self.count_set_value.get())
            print(str(self.current_mass_int))
            self.client.publish(
                "team26/uplink",
                (
                    "CO:"
                    + str(self.count_set_value.get())
                    + "-"
                    + str(self.current_mass_int)
                ),
            )
        else:
            self.ser.write(
                (
                    "CO:"
                    + str(self.count_set_value.get())
                    + "-"
                    + str(self.current_mass_int)
                    + "\n"
                ).encode("utf-8")
            )

    # set pass fail function
    def set_pass_fail(self):
        if self.wifi_mode == True:
            self.client.publish(
                "team26/uplink",
                (
                    "PF:"
                    + str(self.lower_value.get())
                    + "-"
                    + str(self.upper_value.get())
                ),
            )
        else:
            self.ser.write(
                (
                    "PF:"
                    + str(self.lower_value.get())
                    + "-"
                    + str(self.upper_value.get())
                    + "\n"
                ).encode("utf-8")
            )

    # increase brightness function
    def increase_bright(self):
        if self.wifi_mode == True:
            self.client.publish(
                "team26/uplink",
                ("BL:" + "1"),
            )
        else:
            self.ser.write(("BL:" + "1" + "\n").encode("utf-8"))

    # decrease brightness function
    def decrease_bright(self):
        if self.wifi_mode == True:
            self.client.publish(
                "team26/uplink",
                ("BL:" + "0"),
            )
        else:
            self.ser.write(("BL:" + "0" + "\n").encode("utf-8"))

    # open calibration function
    def open_popup(self):

        self.top = Toplevel(root)
        self.cal_weight_entered_one = StringVar(self.top)
        self.cal_weight_entered_one.set(0)
        self.cal_weight_entered_two = StringVar(self.top)
        self.cal_weight_entered_two.set(0)
        self.top.geometry("500x950")
        self.top.title("Calibrate")
        self.instuction1 = Label(
            self.top, text="Take all weight of scale and press NEXT"
        )
        self.instuction1.config(width=50, font=("arial", 10, "bold"))
        self.instuction1.pack(side=TOP, pady=5, expand=True)
        self.next1 = Button(self.top)
        self.next1.config(
            width=9,
            font=("arial", 10, "bold"),
            text="NEXT",
            command=self.zero_set,
        )
        self.next1.pack(side=TOP, pady=10, expand=True)
        self.instuction2 = Label(self.top, text="Place Weight and press NEXT")
        self.instuction2.config(width=50, font=("arial", 10, "bold"))
        self.instuction2.pack(side=TOP, pady=5, expand=True)
        self.next2 = Button(self.top)
        self.next2.config(
            width=9,
            font=("arial", 10, "bold"),
            text="NEXT",
            command=self.first_weigh,
        )
        self.next2.pack(side=TOP, pady=10, expand=True)
        self.instuction3 = Label(self.top, text="Enter Real Weight")
        self.instuction3.config(width=50, font=("arial", 10, "bold"))
        self.instuction3.pack(side=TOP, pady=5, expand=True)

        self.entry1 = Entry(self.top)
        self.entry1.config(
            width=9,
            font=("arial", 10, "bold"),
            textvariable=self.cal_weight_entered_one,
        )
        self.entry1.pack(side=TOP, pady=10, expand=True)
        self.next_weight1 = Label(self.top, text="Next Weight")
        self.next_weight1.config(width=50, font=("arial", 10, "bold"))
        self.next_weight1.pack(side=TOP, pady=5, expand=True)

        self.instuction4 = Label(self.top, text="Place Weight and press NEXT")
        self.instuction4.config(width=50, font=("arial", 10, "bold"))
        self.instuction4.pack(side=TOP, pady=5, expand=True)
        self.next3 = Button(self.top)
        self.next3.config(
            width=9,
            font=("arial", 10, "bold"),
            text="NEXT",
            command=self.second_weigh,
        )
        self.next3.pack(side=TOP, pady=10, expand=True)
        self.instuction5 = Label(self.top, text="Enter Real Weight")
        self.instuction5.config(width=50, font=("arial", 10, "bold"))
        self.instuction5.pack(side=TOP, pady=5, expand=True)

        self.entry2 = Entry(self.top)
        self.entry2.config(
            width=9,
            font=("arial", 10, "bold"),
            textvariable=self.cal_weight_entered_two,
        )
        self.entry2.pack(side=TOP, pady=5, expand=True)

        self.next3 = Button(self.top)
        self.next3.config(
            width=9,
            font=("arial", 10, "bold"),
            text="FINISH",
            command=self.finish_cal,
        )
        self.next3.pack(side=TOP, pady=5, expand=True)

    # internal calibration functions
    def zero_set(self):
        # SET ZEro WEIGHT
        self.cal_zero_adc_zero = self.current_reading.get()

    # internal calibration functions
    def first_weigh(self):
        # SET ZEro WEIGHT
        self.cal_one_adc_one = self.current_reading.get()

    # internal calibration functions
    def second_weigh(self):
        # SET ZEro WEIGHT
        self.cal_one_adc_two = self.current_reading.get()

    # internal calibration functions
    def finish_cal(self):
        # SET ZEro WEIGHT
        print(
            "CA:"
            + self.cal_zero_adc_zero.strip()
            + "-"
            + self.cal_one_adc_one.strip()
            + "-"
            + self.cal_one_adc_two.strip()
            + "-"
            + self.cal_weight_entered_one.get()
            + "-"
            + self.cal_weight_entered_two.get()
            + "\n"
        )
        if self.wifi_mode == True:
            self.client.publish(
                "team26/uplink",
                (
                    "CA:"
                    + self.cal_zero_adc_zero.strip()
                    + "-"
                    + self.cal_one_adc_one.strip()
                    + "-"
                    + self.cal_one_adc_two.strip()
                    + "-"
                    + self.cal_weight_entered_one.get()
                    + "-"
                    + self.cal_weight_entered_two.get()
                ),
            )
        else:
            self.ser.write(
                (
                    "CA:"
                    + self.cal_zero_adc_zero.strip()
                    + "-"
                    + self.cal_one_adc_one.strip()
                    + "-"
                    + self.cal_one_adc_two.strip()
                    + "-"
                    + self.cal_weight_entered_one.get()
                    + "-"
                    + self.cal_weight_entered_two.get()
                    + "\n"
                ).encode("utf-8")
            )

    # SERIAL SETTING VALUE FUNCTIONS

    # chooses function based on the serial input
    def read_serial(self, current_serial):
        current_serial = str(current_serial).replace("\0", "")
        code = current_serial.split(":")
        if code[0] == "MD":
            if code[1].strip() == "0":
                self.current_mode.set("Weigh Mode")
                self.pass_value.set("--")
                # WEIGH
                self.button_select_weigh.config(bg="green")
                self.button_select_count.config(bg="red")
                self.button_select_pass_fail.config(bg="red")
                self.button_tare.config(state="normal")
                self.button_tare_off.config(state="normal")

                # COUNTING
                self.button_enter_count.config(state="disable")
                self.button_set_counting.config(state="disable")

                # PASS FAIL
                self.button_enter_lower.config(state="disable")
                self.button_enter_upper.config(state="disable")
                self.button_set_pass_fail.config(state="disable")
                self.display_pass_fail.config(fg="black")

            elif code[1].strip() == "4":
                self.current_mode.set("Counting Mode")
                self.pass_value.set("--")

                # WEIGH
                self.button_select_weigh.config(bg="red")
                self.button_select_count.config(bg="green")
                self.button_select_pass_fail.config(bg="red")
                self.button_tare.config(state="disable")
                self.button_tare_off.config(state="disable")

                # COUNTING
                self.button_enter_count.config(state="normal")
                self.button_set_counting.config(state="normal")

                # PASS FAIL
                self.button_enter_lower.config(state="disable")
                self.button_enter_upper.config(state="disable")
                self.button_set_pass_fail.config(state="disable")
                self.display_pass_fail.config(fg="black")
            elif code[1].strip() == "2":
                self.current_mode.set("Pass/Fail Mode")

                self.button_select_weigh.config(bg="red")
                self.button_select_count.config(bg="red")
                self.button_select_pass_fail.config(bg="green")
                self.button_tare.config(state="disable")
                self.button_tare_off.config(state="disable")

                # COUNTING
                self.button_enter_count.config(state="disable")
                self.button_set_counting.config(state="disable")

                # PASS FAIL
                self.button_enter_lower.config(state="normal")
                self.button_enter_upper.config(state="normal")
                self.button_set_pass_fail.config(state="normal")

        elif code[0] == "ADC":
            self.current_reading.set((code[1]).replace("\n", ""))
        elif code[0] == "MASS":
            self.current_mass_int = round(
                float((code[1]).replace("\n", "").replace("\r", "")), 1
            )
            self.raw_reading.set(
                str(round(float((code[1]).replace("\n", "").replace("\r", "")), 1))
                + "g"
            )
        elif code[0] == "TA":
            val = code[1].split("-")
            if val[0] == "1":
                self.button_tare.config(bg="green")
                self.button_tare_off.config(bg="red")
            elif val[0] == "0":
                self.button_tare.config(bg="red")
                self.button_tare_off.config(bg="green")
            self.tare_value.set((val[1]).replace("\n", ""))
        elif code[0] == "CV":
            self.count_value.set(str(round((float(code[1].strip())), 2)))
        elif code[0] == "PF":
            val = code[1].split("-")
            self.lower_value.set(str(round((float(val[0].strip())), 2)))
            self.upper_value.set(str(round((float(val[1].strip())), 2)))
        elif code[0] == "PV":
            if code[1].strip() == "1":
                self.pass_value.set("PASS")
                self.display_pass_fail.config(fg="green")
            elif code[1].strip() == "0":
                self.pass_value.set("FAIL")
                self.display_pass_fail.config(fg="red")
        elif code[0] == "PO":
            self.connected = True
        elif code[0] == "ZV":
            self.zero_weight.set("Z:" + code[1].strip())
        elif code[0] == "CM":
            if code[1].strip() == "1":
                if self.wifi_mode == True:
                    pass
                elif self.wifi_mode == False:
                    self.wifi_on()
            if code[1].strip() == "0":
                if self.wifi_mode == False:
                    pass
                elif self.wifi_mode == True:
                    self.wifi_on()
        elif code[0] == "OL":
            self.raw_reading.set("OL")
        elif code[0] == "CO":
            val = code[1].split("-")
            self.count_set_value.set(val[0].strip())
            self.ref_mass.set(str(round((float(val[1].strip())), 2)) + "g")
            self.unit_mass.set(str(round((float(val[2].strip())), 2)) + "g")

        else:
            print(code[0])

    # WIFI function

    # SERIAL COMS FUNCTIONS

    # Opens the Serial port for communication
    def open_serial(self):
        i = 1
        while i != 0:
            try:
                self.ser = serial.Serial(self.port, 115200)
                self.t = threading.Thread(target=self.thread_read_serial)
                self.t.start()
                i = 0
            except serial.serialutil.SerialException:
                self.ser = None

    # Thread function for reading from the serial port
    def thread_read_serial(self):
        while True:
            try:
                if self.connected == False:
                    self.ser.write(("PO:" + "1" + "\n").encode("utf-8"))
            except serial.serialutil.SerialException:
                self.open = False
            if self.wifi_mode == False:
                try:
                    current_serial = self.ser.readline().decode("utf-8")
                    self.read_serial(current_serial.replace("\0", ""))
                except serial.serialutil.SerialException:
                    self.open = False
                    try:
                        self.ser = serial.Serial(self.port, 115200)
                        self.ser.write(("PO:" + "1" + "\n").encode("utf-8"))
                    except serial.serialutil.SerialException:
                        pass

            else:
                continue

    # Thread function for openiing the serial port
    def thread_open_serial(self):
        while self.open != True:
            self.ports = list(serial.tools.list_ports.comports())
            for p in self.ports:
                if "USB Serial" in p.description:
                    self.port = p[0]
                    self.open = True
                    break
        self.ser = None
        self.open_serial()

    def run(self):
        self.client = mqtt.Client(transport="websockets")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        username = "team26"
        password = "e5aabb21296d9254786e9aa0"
        self.client.tls_set(cert_reqs=ssl.CERT_NONE)
        self.client.ws_set_options(path="/ws")
        self.client.username_pw_set(username, password)

        self.client.connect("tp-mqtt.uqcloud.net", 443, 60)

        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        client.connected_flag = True
        client.subscribe("team26/downlink")

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self, client, userdata, msg):
        if self.wifi_mode == True:
            message = str(msg.payload).split("'")
            self.read_serial(message[1])


e = Elder(root)


root.mainloop()
