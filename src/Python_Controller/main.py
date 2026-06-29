import dearpygui.dearpygui as dpg
import os
import platform

# Front
def create_fonts():
    system = platform.system()

    if system == "Windows":
        possible_fonts = [
            r"C:\Windows\Fonts\arial.ttf"
        ]
    elif system == "Darwin":
        possible_fonts = [
            "/System/Library/Fonts/Supplemental/Arial.ttf",
            "/Library/Fonts/Arial.ttf"
        ]
    else:
        possible_fonts = []

    font_path = next((p for p in possible_fonts if os.path.exists(p)), None)

    if font_path is None:
        raise FileNotFoundError("Could not find a usable font file.")

    with dpg.font_registry():
        fonts = {
            "title": dpg.add_font(font_path, 30),
            "small": dpg.add_font(font_path, 18),
            "medium": dpg.add_font(font_path, 23),
            "large": dpg.add_font(font_path, 40),
        }

    return fonts


def apply_font(item_tag, font):
    dpg.bind_item_font(item_tag, font)


# Window theme
def theme_main_window():
    with dpg.theme() as window_theme:
        with dpg.theme_component(dpg.mvWindowAppItem):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (255, 255, 255, 255))
        with dpg.theme_component(dpg.mvText):
            dpg.add_theme_color(dpg.mvThemeCol_Text, (0, 0, 0, 255))
    dpg.bind_item_theme("main_window", window_theme)


# Button Color, Size
def theme_stop_button():
    with dpg.theme() as button_theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (242, 165, 48, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 0, 0, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (0, 0, 0, 255))

    dpg.bind_item_theme("stop_button", button_theme)
    dpg.bind_item_theme("start_button", button_theme)

    with dpg.theme() as button_theme2:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (48, 81, 242, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 0, 0, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))

    dpg.bind_item_theme("e_stop_button", button_theme2)
    dpg.bind_item_theme("choose_output_button", button_theme2)

def theme_toggle_hitbox():
    with dpg.theme() as invisible_theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (0, 0, 0, 0))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (0, 0, 0, 0))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (0, 0, 0, 0))
            dpg.add_theme_color(dpg.mvThemeCol_Border, (0, 0, 0, 0))

    dpg.bind_item_theme("toggle_hitbox1", invisible_theme)
    dpg.bind_item_theme("toggle_hitbox2", invisible_theme)
    dpg.bind_item_theme("toggle_hitbox3", invisible_theme)
    dpg.bind_item_theme("toggle_hitbox4", invisible_theme)

def theme_power_slider():
    with dpg.theme() as slider_theme:
        with dpg.theme_component(dpg.mvSliderFloat):
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (50, 50, 50, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (60, 60, 60, 255))
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrab, (48, 81, 242, 255))
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrabActive, (255, 0, 0, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))

    dpg.bind_item_theme("power_slider", slider_theme)

#Image Upload
"""
def load_texture(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Image not found: {path}")

    image_data = dpg.load_image(path)
    if image_data is None:
        raise ValueError(f"Dear PyGui failed to load image: {path}")

    width, height, channels, data = image_data
    return dpg.add_static_texture(width, height, data)
"""
# Main function
def main():
    dpg.create_context()
    dpg.create_viewport(title="Solar Gamera", width=1500, height=1000)
    dpg.setup_dearpygui()

    fonts = create_fonts()

    with dpg.texture_registry():
        # Better to use an English-only path if possible
        image_path = "D:/image.png" # change the image storage path

    with dpg.window(tag="main_window"):
        dpg.add_text("Solar Gamera", tag="title_text", pos=(30, 20))
        """
        with dpg.texture_registry():
            image_path = "/Users/yxt/Desktop/image.png"  # change to your Mac path
    
        if os.path.exists(image_path):
            my_texture = load_texture(image_path)
        else:
            print(f"Warning: Image not found → {image_path}")
            my_texture = None
        """

        with dpg.plot(label="PPM and Throttle% vs time (N)", height=280, width=390, pos=(650, 310)):
            dpg.add_plot_legend()
            dpg.add_plot_axis(dpg.mvXAxis, label="Time")
            y_axis_n = dpg.add_plot_axis(dpg.mvYAxis, label="RPM")
            dpg.add_line_series([], [], parent=y_axis_n, tag="rpm_series_n", label="Live Data")
        
        with dpg.plot(label="PPM and Throttle% vs time (S)", height=280, width=390, pos=(1040, 310)):
            dpg.add_plot_legend()
            dpg.add_plot_axis(dpg.mvXAxis, label="Time")
            y_axis_s = dpg.add_plot_axis(dpg.mvYAxis, label="RPM")
            dpg.add_line_series([], [], parent=y_axis_s, tag="rpm_series_s", label="Live Data")
        
        with dpg.plot(label="PPM and Throttle% vs time (E)", height=280, width=390, pos=(650, 590)):
            dpg.add_plot_legend()
            dpg.add_plot_axis(dpg.mvXAxis, label="Time")
            y_axis_e = dpg.add_plot_axis(dpg.mvYAxis, label="RPM")
            dpg.add_line_series([], [], parent=y_axis_e, tag="rpm_series_e", label="Live Data")
        
        with dpg.plot(label="PPM and Throttle% vs time (W)", height=280, width=390, pos=(1040, 590)):
            dpg.add_plot_legend()
            dpg.add_plot_axis(dpg.mvXAxis, label="Time")
            y_axis_w = dpg.add_plot_axis(dpg.mvYAxis, label="RPM")
            dpg.add_line_series([], [], parent=y_axis_w, tag="rpm_series_w", label="Live Data")

        """
        dpg.set_value("rpm_series_n", [ ])
        dpg.set_value("rpm_series_s", [ ])
        dpg.set_value("rpm_series_e", [ ])
        dpg.set_value("rpm_series_w", [ ])
        """
        #dpg.add_button(label="Add Data", callback=add_point, pos=(650, 870))


        # file outputing 

        with dpg.file_dialog(
            directory_selector=False,
            show=False,
            callback=save_file_callback,
            tag="save_file_dialog",
            width=700,
            height=400,
            default_filename="output.txt"
        ):
            dpg.add_file_extension(".txt")
            dpg.add_file_extension(".csv")


        dpg.add_button(
            label="Choose Output File",
            tag="choose_output_button",
            width=780,
            height=30,
            pos=(650, 870),
            callback=open_save_dialog
        )


        # Emergency Stop Button
        dpg.add_button(label="EMERGENCY STOP",
            tag="stop_button",
            callback=Emerstop,
            width=620,
            height=80,
            pos=(650, 218)
        )

        # E-STOP Reset
        dpg.add_button(label="E-STOP Reset",
            tag="e_stop_button",
            callback=estop,
            width=160,
            height=80,
            pos=(1270, 218),
        )

        dpg.draw_rectangle(
            (10, 303),
            (510, 350),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )

        # RSSI
        RSSI = RSSI_val( )
        dpg.draw_text(
            (20, 315),
            RSSI,
            tag="RSSI_text",
            size=25,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        # LQ
        LQ = LQ_val( )
        dpg.draw_text(
            (225, 315),
            LQ,
            tag="LQ_text",
            size=25,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        # SNR
        SNR = SNR_val( )
        dpg.draw_text(
            (380, 315),
            SNR,
            tag="SNR_text",
            size=25,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        # Toggle Button 1
        dpg.add_button(
            label="",
            tag="toggle_hitbox1",
            width=150,
            height=300,
            pos=(650, 20),
            callback=toggle1
        )

        # Toggle Button 2
        dpg.add_button(
            label="",
            tag="toggle_hitbox2",
            width=150,
            height=300,
            pos=(850, 20),
            callback=toggle2
        )

        # Toggle Button 3
        dpg.add_button(
            label="",
            tag="toggle_hitbox3",
            width=150,
            height=300,
            pos=(1050, 20),
            callback=toggle3
        )

        # Toggle Button 4
        dpg.add_button(
            label="",
            tag="toggle_hitbox4",
            width=150,
            height=300,
            pos=(1250, 20),
            callback=toggle4
        )

        # Percentage and RPM 1
        dpg.draw_rectangle(
            (642, 160),
            (700, 200),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )

        dpg.draw_rectangle(
            (710, 160),
            (793, 200),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )

        perc1 = angvel1()[0]

        dpg.draw_text(
            (647, 168),
            perc1,
            tag="rpm_text",
            size=20,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        rpm1 = angvel1()[1]

        dpg.draw_text(
            (718, 168),
            rpm1,
            tag="rpm_text1",
            size=20,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        dpg.draw_rectangle(
            (642, 120),
            (793, 150),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )
        dpg.draw_text(
            (710, 127),
            "N",
            tag="rpm_text2",
            size=17,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        # Percentage and RPM 2
        dpg.draw_rectangle(
            (852, 160),
            (912, 200),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )

        dpg.draw_rectangle(
            (920, 160),
            (1003, 200),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )

        perc2 = angvel2()[0]

        dpg.draw_text(
            (857, 168),
            perc2,
            tag="rpm_text3",
            size=20,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        rpm2 = angvel2()[1]

        dpg.draw_text(
            (928, 168),
            rpm2,
            tag="rpm_text4",
            size=20,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        dpg.draw_rectangle(
            (852, 120),
            (1003, 150),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )
        dpg.draw_text(
            (920, 127),
            "E",
            tag="rpm_text5",
            size=17,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        # Percentage and RPM 3
        dpg.draw_rectangle(
            (1062, 160),
            (1122, 200),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )

        dpg.draw_rectangle(
            (1130, 160),
            (1213, 200),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )

        perc3 = angvel3()[0]

        dpg.draw_text(
            (1067, 168),
            perc3,
            tag="rpm_text6",
            size=20,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        rpm3 = angvel3()[1]

        dpg.draw_text(
            (1138, 168),
            rpm3,
            tag="rpm_text7",
            size=20,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        dpg.draw_rectangle(
            (1062, 120),
            (1213, 150),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )
        dpg.draw_text(
            (1130, 127),
            "S",
            tag="rpm_text8",
            size=17,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        # Percentage and RPM 4
        dpg.draw_rectangle(
            (1272, 160),
            (1332, 200),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )

        dpg.draw_rectangle(
            (1340, 160),
            (1423, 200),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )

        perc4 = angvel4()[0]

        dpg.draw_text(
            (1277, 168),
            perc4,
            tag="rpm_text9",
            size=20,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        rpm4 = angvel4()[1]

        dpg.draw_text(
            (1348, 168),
            rpm4,
            tag="rpm_text10",
            size=20,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        dpg.draw_rectangle(
            (1272, 120),
            (1423, 150),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),  # white
            parent="value_layer"
        )
        dpg.draw_text(
            (1340, 127),
            "W",
            tag="rpm_text11",
            size=17,
            color=(0, 0, 0, 255),
            parent="value_layer"
        )

        # vertical slider
        
        power_percent = 0
        start = False #initialize
        
        dpg.add_slider_float(
            tag="power_slider",
            default_value=0,
            min_value=0,
            max_value=100,
            vertical=True,
            pos=(530, 310),
            width=100,
            height=540,
            callback=power_slider_fun
        )
        theme_power_slider()

        # Start Motors button
        dpg.add_button(
            label="Start",
            tag="start_button",
            width=100,
            height=50,
            pos=(530, 850),
            callback=start_button_fun
        )

    with dpg.viewport_drawlist(front=True, tag="toggle_layer"):
        pass



    redraw_toggles()

    apply_font("title_text", fonts["title"])
    apply_font("stop_button", fonts["large"])
    apply_font("e_stop_button", fonts["medium"])
    apply_font("RSSI_text", fonts["title"])
    apply_font("LQ_text", fonts["title"])
    apply_font("SNR_text", fonts["title"])
    apply_font("rpm_text", fonts["title"])
    apply_font("rpm_text1", fonts["title"])
    apply_font("rpm_text2", fonts["title"])
    apply_font("rpm_text3", fonts["title"])
    apply_font("rpm_text4", fonts["title"])
    apply_font("rpm_text5", fonts["title"])
    apply_font("rpm_text6", fonts["title"])
    apply_font("rpm_text7", fonts["title"])
    apply_font("rpm_text8", fonts["title"])
    apply_font("rpm_text9", fonts["title"])
    apply_font("rpm_text10", fonts["title"])
    apply_font("rpm_text11", fonts["title"])
    apply_font("start_button", fonts["medium"])

    theme_main_window()
    theme_stop_button()
    theme_toggle_hitbox()

    dpg.set_primary_window("main_window", True)

    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()

# graph
pwm_data = []
rpm_data = []

def open_save_dialog(sender, app_data):
    dpg.show_item("save_file_dialog")

output_file_path = ""

def save_file_callback(sender, app_data):
    global output_file_path

    # app_data["file_path_name"] is the full selected path
    output_file_path = app_data["file_path_name"]
    print("Output file path:", output_file_path)


# Emergence Stop function
def Emerstop():
    print("EMERGENCY STOP pressed")

def estop():
    print("E-STOP pressed")

def RSSI_val():
    val = 5
    ouput = f"RSSI = -{val}/5dBm"
    return ouput

def LQ_val():
    val = 100
    ouput = f"LQ = {val}%"
    return ouput

def SNR_val():
    val = 12
    ouput = f"LQ = {val}db"
    return ouput

# Toggle Button initial state
toggle_state1 = {"up": False}
toggle_state2 = {"up": False}
toggle_state3 = {"up": False}
toggle_state4 = {"up": False}

# Toggle Button Moving Function
def draw_toggle(TOGGLE_X, TOGGLE_Y, state, hitbox_tag, text_tag):
    TOGGLE_W = 150
    TOGGLE_H = 100

    dpg.draw_rectangle(
        (TOGGLE_X, TOGGLE_Y),
        (TOGGLE_X + TOGGLE_W, TOGGLE_Y + TOGGLE_H),
        color=(0, 0, 0, 255),
        thickness=1,
        parent="toggle_layer"
    )

    if state["up"]: #right active
        x1 = TOGGLE_X + 15 - 2 + 50 
        x2 = TOGGLE_X + 100 - 10 -2 + 50
        text_x = 78
        border_col = (120, 180, 120, 255)
        fill_col = (48, 81, 242, 255)
    else: #left inactive
        x1 = TOGGLE_X + 15 - 2
        x2 = TOGGLE_X + 100 - 10 -2
        text_x = 25
        border_col = (200, 200, 200, 255)
        fill_col = (200, 200, 200, 255)

    dpg.draw_rectangle(
        (x1 , TOGGLE_Y + 10),
        (x2, TOGGLE_Y + 90),
        fill=fill_col,
        color=border_col,
        thickness=1,
        parent="toggle_layer"
    )

    dpg.draw_text(
        (TOGGLE_X + text_x, TOGGLE_Y + 50),
        "Toggle",
        color=(255, 255, 255, 255),
        size=15,
        parent="toggle_layer",
        tag=text_tag
    )

    dpg.set_item_pos(hitbox_tag, (TOGGLE_X, TOGGLE_Y))

# Reset toggle button postion every click
def redraw_toggles():
    dpg.delete_item("toggle_layer", children_only=True)
    draw_toggle(650, 20, toggle_state1, "toggle_hitbox1", "toggletext1")
    draw_toggle(860, 20, toggle_state2, "toggle_hitbox2", "toggletext2")
    draw_toggle(1070, 20, toggle_state3, "toggle_hitbox3", "toggletext3")
    draw_toggle(1280, 20, toggle_state4, "toggle_hitbox4", "toggletext4")


# Toggle Button 1 Function
def toggle1():
    toggle_state1["up"] = not toggle_state1["up"]
    redraw_toggles()
    print("Toggle 1 Pressed")

# Toggle Button 2 Function
def toggle2():
    toggle_state2["up"] = not toggle_state2["up"]
    redraw_toggles()
    print("Toggle 2 Pressed")

# Toggle Button 3 Function
def toggle3():
    toggle_state3["up"] = not toggle_state3["up"]
    redraw_toggles()
    print("Toggle 3 Pressed")

# Toggle Button 4 Function
def toggle4():
    toggle_state4["up"] = not toggle_state4["up"]
    redraw_toggles()
    print("Toggle 4 Pressed")

# Angular velocity box
def angvel1():
    rpm = f"{37} RPM"
    perc = f"{37} %"
    return perc, rpm

def angvel2():
    rpm = f"{27} RPM"
    perc = f"{100} %"
    return perc, rpm

def angvel3():
    rpm = f"{27} RPM"
    perc = f"{100} %"
    return perc, rpm

def angvel4():
    rpm = f"{17} RPM"
    perc = f"{50} %"
    return perc, rpm


# Set power input
def power_slider_fun(sender, app_data):
    global power_percent
    power_percent = app_data

def start_button_fun():
    global start
    start = True
    print("Ouput power percentage =", power_percent)


if __name__ == "__main__":
    main()
