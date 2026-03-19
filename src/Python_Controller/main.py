import dearpygui.dearpygui as dpg
import os

# Front
def create_fonts():
    with dpg.font_registry():
        fonts = {
            "title": dpg.add_font("C:/Windows/Fonts/arial.ttf", 30),
            "small": dpg.add_font("C:/Windows/Fonts/arial.ttf", 18),
            "medium": dpg.add_font("C:/Windows/Fonts/arial.ttf", 24),
            "large": dpg.add_font("C:/Windows/Fonts/arial.ttf", 40),
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
            dpg.add_theme_color(dpg.mvThemeCol_Button, (241, 193, 193, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (230, 175, 175, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (220, 160, 160, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (0, 0, 0, 255))
    dpg.bind_item_theme("stop_button", button_theme)

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


#Image Upload
def load_texture(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Image not found: {path}")

    image_data = dpg.load_image(path)
    if image_data is None:
        raise ValueError(f"Dear PyGui failed to load image: {path}")

    width, height, channels, data = image_data
    return dpg.add_static_texture(width, height, data)

# Main function
def main():
    dpg.create_context()
    dpg.create_viewport(title="Solar Gamera", width=1500, height=1000)
    dpg.setup_dearpygui()

    fonts = create_fonts()

    with dpg.texture_registry():
        # Better to use an English-only path if possible
        image_path = "D:/image.png"
        my_texture = load_texture(image_path)

    with dpg.window(tag="main_window"):
        dpg.add_text("Solar Gamera", tag="title_text", pos=(30, 20))
        dpg.add_image(my_texture, pos=(80, 50), width=400, height=400)


        # Emergency Stop Button
        dpg.add_button(
            label="EMERGENCY STOP",
            tag="stop_button",
            callback=Emerstop,
            width=800,
            height=100,
            pos=(650, 430)
        )

        # Toggle Button 1
        dpg.add_button(
            label="",
            tag="toggle_hitbox1",
            width=150,
            height=300,
            pos=(650, 40),
            callback=toggle1
        )

        # Toggle Button 2
        dpg.add_button(
            label="",
            tag="toggle_hitbox2",
            width=150,
            height=300,
            pos=(850, 40),
            callback=toggle2
        )

        # Toggle Button 3
        dpg.add_button(
            label="",
            tag="toggle_hitbox3",
            width=150,
            height=300,
            pos=(1050, 40),
            callback=toggle3
        )

        # Toggle Button 4
        dpg.add_button(
            label="",
            tag="toggle_hitbox4",
            width=150,
            height=300,
            pos=(1250, 40),
            callback=toggle4
        )

    with dpg.viewport_drawlist(front=True, tag="toggle_layer"):
        pass

    redraw_toggles()

    apply_font("title_text", fonts["title"])
    apply_font("stop_button", fonts["large"])

    theme_main_window()
    theme_stop_button()
    theme_toggle_hitbox()

    dpg.set_primary_window("main_window", True)

    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()


# Emergence Stop function
def Emerstop():
    print("EMERGENCY STOP pressed")


# Toggle Button initial state
toggle_state1 = {"up": False}
toggle_state2 = {"up": False}
toggle_state3 = {"up": False}
toggle_state4 = {"up": False}

# Toggle Button Moving Function
def draw_toggle(TOGGLE_X, TOGGLE_Y, state, hitbox_tag, text_tag):
    TOGGLE_W = 150
    TOGGLE_H = 300

    dpg.draw_rectangle(
        (TOGGLE_X, TOGGLE_Y),
        (TOGGLE_X + TOGGLE_W, TOGGLE_Y + TOGGLE_H),
        color=(0, 0, 0, 255),
        thickness=1,
        parent="toggle_layer"
    )

    if state["up"]:
        y1 = TOGGLE_Y + 40
        y2 = TOGGLE_Y + 180
        text_y = TOGGLE_Y + 95
        border_col = (200, 200, 200, 255)
        fill_col = (200, 200, 200, 255)
    else:
        y1 = TOGGLE_Y + 140
        y2 = TOGGLE_Y + 280
        text_y = TOGGLE_Y + 215
        border_col = (120, 180, 120, 255)
        fill_col = (210, 225, 205, 255)

    dpg.draw_rectangle(
        (TOGGLE_X + 15, y1),
        (TOGGLE_X + 150 - 10, y2 - 15),
        fill=fill_col,
        color=border_col,
        thickness=1,
        parent="toggle_layer"
    )

    dpg.draw_text(
        (TOGGLE_X + 18, text_y),
        "Press to toggle",
        color=(0, 0, 0, 255),
        size=15,
        parent="toggle_layer",
        tag=text_tag
    )

    dpg.set_item_pos(hitbox_tag, (TOGGLE_X, TOGGLE_Y))

# Reset Buttom postion every click
def redraw_toggles():
    dpg.delete_item("toggle_layer", children_only=True)
    draw_toggle(650, 40, toggle_state1, "toggle_hitbox1", "toggletext1")
    draw_toggle(850, 40, toggle_state2, "toggle_hitbox2", "toggletext2")
    draw_toggle(1050, 40, toggle_state3, "toggle_hitbox3", "toggletext3")
    draw_toggle(1250, 40, toggle_state4, "toggle_hitbox4", "toggletext4")


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


if __name__ == "__main__":
    main()
