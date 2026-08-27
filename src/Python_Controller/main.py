import dearpygui.dearpygui as dpg
import os
import platform


ARM_SPECS = (
    ("n", "N", 37.0, 37.0),
    ("e", "E", 100.0, 27.0),
    ("s", "S", 100.0, 27.0),
    ("w", "W", 50.0, 17.0),
)

# Layout constraints. Screen positions are intentionally not included here;
# containers determine placement and the resize callback only adjusts heights.
DEFAULT_VIEWPORT_WIDTH = 1500
DEFAULT_VIEWPORT_HEIGHT = 1000
TITLE_COLUMN_WIDTH = 220
ARM_CARD_HEIGHT = 190
TOGGLE_HEIGHT = 100
VALUE_ROW_HEIGHT = 40
ESTOP_ROW_HEIGHT = 200
ESTOP_RESET_ROW_HEIGHT = 100
UI_GUTTER = 9
# Vertical gaps between consecutive rows in the side control stack. These are
# passed directly to Dear PyGui, so negative values are valid when needed.
THROTTLE_LIMIT_ROW_GAP = 9
THROTTLE_SET_ROW_GAP = -11
THROTTLE_TO_ARM_ROW_GAP = -11
ARM_TO_ESTOP_ROW_GAP = 9

# Graph-only margins. Unlike body-table padding, these do not affect the
# side-control panel.
GRAPH_SECTION_TOP_MARGIN = UI_GUTTER
GRAPH_SECTION_BOTTOM_MARGIN = 5
# Visible gaps between adjacent plots in the 2 × 2 graph grid.
GRAPH_COLUMN_GUTTER = UI_GUTTER
GRAPH_ROW_GUTTER = UI_GUTTER

THROTTLE_CONTROL_ROW_HEIGHT = 70
INTEGER_INPUT_VERTICAL_PADDING = 24
# The action row absorbs Dear PyGui's three pixels of trailing table padding so
# its visible bottom gutter matches UI_GUTTER.
START_BUTTON_HEIGHT = 80
OUTPUT_BUTTON_HEIGHT = START_BUTTON_HEIGHT
MIN_PLOT_HEIGHT = 160
# Space used above/below the plots by the header, action row, and padding.
# Includes the fixed header/action rows, table spacing, and the main window's
# bottom padding so the content fits without creating a vertical scrollbar.
LAYOUT_VERTICAL_OVERHEAD = 294


class ArmCard:
    """Reusable controls and readouts for one helicopter arm."""

    def __init__(
        self,
        arm_id,
        label,
        throttle_percent=0.0,
        rpm=0.0,
        toggle_themes=None,
        font=None,
    ):
        self.arm_id = arm_id
        self.label = label
        self.enabled = False
        self.throttle_percent = throttle_percent
        self.rpm = rpm
        self.toggle_themes = toggle_themes or {}
        self.font = font

        self.card_tag = f"arm_{arm_id}_card"
        self.toggle_drawlist_tag = f"arm_{arm_id}_toggle_drawlist"
        self.toggle_handler_tag = f"arm_{arm_id}_toggle_handlers"
        self.resize_handler_tag = f"arm_{arm_id}_resize_handlers"
        self.label_tag = f"arm_{arm_id}_label"
        self.throttle_tag = f"arm_{arm_id}_throttle"
        self.rpm_tag = f"arm_{arm_id}_rpm"

    def build(self, parent):
        with dpg.child_window(
            tag=self.card_tag,
            parent=parent,
            width=-1,
            height=ARM_CARD_HEIGHT,
            border=False,
            no_scrollbar=True,
            no_scroll_with_mouse=True,
        ):
            dpg.add_drawlist(
                tag=self.toggle_drawlist_tag,
                # Drawlists need a concrete canvas size. This is replaced by
                # the stretched card width after the first layout frame.
                width=150,
                height=TOGGLE_HEIGHT,
            )
            dpg.add_button(
                label=self.label,
                tag=self.label_tag,
                width=-1,
                height=30,
                enabled=False,
            )
            with dpg.table(
                header_row=False,
                policy=dpg.mvTable_SizingStretchSame,
                width=-1,
                no_pad_outerX=True,
            ):
                dpg.add_table_column(width_stretch=True)
                dpg.add_table_column(width_stretch=True)
                with dpg.table_row():
                    with dpg.table_cell():
                        dpg.add_button(
                            label=self._format_throttle(),
                            tag=self.throttle_tag,
                            width=-1,
                            height=VALUE_ROW_HEIGHT,
                            enabled=False,
                        )
                    with dpg.table_cell():
                        dpg.add_button(
                            label=self._format_rpm(),
                            tag=self.rpm_tag,
                            width=-1,
                            height=VALUE_ROW_HEIGHT,
                            enabled=False,
                        )

        with dpg.item_handler_registry(tag=self.toggle_handler_tag):
            dpg.add_item_clicked_handler(callback=self.toggle)
        dpg.bind_item_handler_registry(
            self.toggle_drawlist_tag,
            self.toggle_handler_tag,
        )

        with dpg.item_handler_registry(tag=self.resize_handler_tag):
            dpg.add_item_resize_handler(callback=self._on_resize)
        dpg.bind_item_handler_registry(self.card_tag, self.resize_handler_tag)

        self._apply_shared_style()
        self._redraw_toggle()

    def toggle(self, sender=None, app_data=None, user_data=None):
        self.enabled = not self.enabled
        self._redraw_toggle()
        print(f"Arm {self.label} toggle {'ON' if self.enabled else 'OFF'}")

    def _on_resize(self, sender=None, app_data=None, user_data=None):
        self._redraw_toggle()

    def set_throttle(self, percent):
        self.throttle_percent = percent
        dpg.configure_item(self.throttle_tag, label=self._format_throttle())

    def set_rpm(self, rpm):
        self.rpm = rpm
        dpg.configure_item(self.rpm_tag, label=self._format_rpm())

    def _format_throttle(self):
        return f"{self.throttle_percent:g} %"

    def _format_rpm(self):
        return f"{self.rpm:g} RPM"

    def _apply_shared_style(self):
        card_theme = self.toggle_themes.get("card")
        if card_theme is not None:
            dpg.bind_item_theme(self.card_tag, card_theme)

        value_theme = self.toggle_themes.get("value")
        for tag in (self.label_tag, self.throttle_tag, self.rpm_tag):
            if value_theme is not None:
                dpg.bind_item_theme(tag, value_theme)
            if self.font is not None:
                dpg.bind_item_font(tag, self.font)

    def _redraw_toggle(self):
        dpg.delete_item(self.toggle_drawlist_tag, children_only=True)

        card_width = dpg.get_item_rect_size(self.card_tag)[0]
        if card_width > 1:
            draw_width = round(card_width)
            dpg.configure_item(self.toggle_drawlist_tag, width=draw_width)
        else:
            draw_width = 150

        margin = max(8, round(draw_width * 0.08))
        knob_width = max(50, round(draw_width * 0.5))

        dpg.draw_rectangle(
            (0, 0),
            (draw_width - 1, TOGGLE_HEIGHT - 1),
            color=(0, 0, 0, 255),
            fill=(255, 255, 255, 255),
            parent=self.toggle_drawlist_tag,
        )

        if self.enabled:
            knob_right = draw_width - margin
            knob_left = knob_right - knob_width
            knob_color = (48, 81, 242, 255)
        else:
            knob_left = margin
            knob_right = knob_left + knob_width
            knob_color = (200, 200, 200, 255)

        knob_min = (knob_left, 10)
        knob_max = (knob_right, TOGGLE_HEIGHT - 10)
        text_pos = ((knob_left + knob_right) / 2 - 22, 43)

        dpg.draw_rectangle(
            knob_min,
            knob_max,
            color=knob_color,
            fill=knob_color,
            parent=self.toggle_drawlist_tag,
        )
        dpg.draw_text(
            text_pos,
            "Toggle",
            color=(255, 255, 255, 255),
            size=15,
            parent=self.toggle_drawlist_tag,
        )

arm_cards = {}


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
            "lock": dpg.add_font(font_path, 12),
            "small": dpg.add_font(font_path, 18),
            "medium": dpg.add_font(font_path, 23),
            "estop": dpg.add_font(font_path, 34),
        }

    return fonts


def apply_font(item_tag, font):
    dpg.bind_item_font(item_tag, font)


# Window theme
def theme_main_window():
    with dpg.theme() as window_theme:
        with dpg.theme_component(dpg.mvWindowAppItem):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (255, 255, 255, 255))
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (255, 255, 255, 255))
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
    dpg.bind_item_theme("max_throttle_button", button_theme)
    dpg.bind_item_theme("min_throttle_button", button_theme)
    dpg.bind_item_theme("set_velocity_button", button_theme)
    dpg.bind_item_theme("set_throttle_button", button_theme)

    with dpg.theme() as button_theme2:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (48, 81, 242, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 0, 0, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))

    dpg.bind_item_theme("e_stop_button", button_theme2)
    dpg.bind_item_theme("arm_button", button_theme2)
    dpg.bind_item_theme("throttle_lock_button", button_theme2)
    dpg.bind_item_theme("choose_output_button", button_theme2)

    with dpg.theme() as integer_input_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_style(
                dpg.mvStyleVar_FramePadding,
                4,
                INTEGER_INPUT_VERTICAL_PADDING,
            )

    dpg.bind_item_theme("throttle_velocity_input", integer_input_theme)
    dpg.bind_item_theme("throttle_input", integer_input_theme)

    row_gap_settings = (
        ("throttle_limit_row", THROTTLE_LIMIT_ROW_GAP),
        ("velocity_control_row", THROTTLE_SET_ROW_GAP),
        ("throttle_control_row", THROTTLE_TO_ARM_ROW_GAP),
        ("control_button_row", ARM_TO_ESTOP_ROW_GAP),
    )
    for row_tag, row_gap in row_gap_settings:
        with dpg.theme() as row_gap_theme:
            with dpg.theme_component(dpg.mvTable):
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, row_gap)
        dpg.bind_item_theme(row_tag, row_gap_theme)

    with dpg.theme() as control_spacing_theme:
        with dpg.theme_component(dpg.mvGroup):
            dpg.add_theme_style(
                dpg.mvStyleVar_ItemSpacing,
                8,
                UI_GUTTER,
            )

    dpg.bind_item_theme("control_button_stack", control_spacing_theme)

    with dpg.theme() as graph_section_theme:
        with dpg.theme_component(dpg.mvGroup):
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 0, 0)

    dpg.bind_item_theme("graph_section", graph_section_theme)

    with dpg.theme() as plot_grid_theme:
        with dpg.theme_component(dpg.mvTable):
            # Table padding is applied to both adjacent cells, so each side
            # receives half of the desired visible gutter.
            dpg.add_theme_style(
                dpg.mvStyleVar_CellPadding,
                GRAPH_COLUMN_GUTTER / 2,
                GRAPH_ROW_GUTTER / 2,
            )

    dpg.bind_item_theme("plot_grid", plot_grid_theme)

    with dpg.theme() as body_spacing_theme:
        with dpg.theme_component(dpg.mvTable):
            dpg.add_theme_style(
                dpg.mvStyleVar_CellPadding,
                4,
                0,
            )

    dpg.bind_item_theme("body_layout", body_spacing_theme)

def create_arm_card_themes():
    themes = {}

    with dpg.theme() as themes["card"]:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (255, 255, 255, 255))
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (255, 255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 0, 0)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 0, 4)

    with dpg.theme() as themes["value"]:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (255, 255, 255, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (0, 0, 0, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Border, (0, 0, 0, 255))
            dpg.add_theme_style(dpg.mvStyleVar_DisabledAlpha, 1.0)

    return themes

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
def add_arm_plot(arm_id, label):
    """Build one plot that stretches to the width of its table cell."""
    with dpg.plot(
        label=f"RPM and Throttle% vs time ({label})",
        tag=f"plot_{arm_id}",
        height=MIN_PLOT_HEIGHT,
        width=-1,
    ):
        dpg.add_plot_legend()
        dpg.add_plot_axis(dpg.mvXAxis, label="Time")
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="RPM")
        dpg.add_line_series(
            [],
            [],
            parent=y_axis,
            tag=f"rpm_series_{arm_id}",
            label="Live Data",
        )


def update_responsive_layout(sender=None, app_data=None, user_data=None):
    """Resize height-dependent controls after the viewport changes."""
    viewport_height = dpg.get_viewport_client_height()
    plot_height = max(
        MIN_PLOT_HEIGHT,
        (
            viewport_height
            - LAYOUT_VERTICAL_OVERHEAD
            - GRAPH_SECTION_TOP_MARGIN
            - GRAPH_SECTION_BOTTOM_MARGIN
            # The graph table applies vertical cell padding above, between,
            # and below its two plot rows.
            - GRAPH_ROW_GUTTER * 2
        )
        // 2,
    )

    for arm_id, *_ in ARM_SPECS:
        dpg.configure_item(f"plot_{arm_id}", height=plot_height)

    # The viewport callback runs before Dear ImGui lays out the resized table.
    # Redraw on the following frame, when each card's rendered width is final.
    dpg.set_frame_callback(
        dpg.get_frame_count() + 2,
        refresh_arm_card_layout,
    )


def refresh_arm_card_layout(sender=None, app_data=None, user_data=None):
    """Apply the header table's settled widths to the custom toggle canvases."""
    for card in arm_cards.values():
        card._redraw_toggle()


def main():
    global start, throttle_locked, throttle_percent, throttle_velocity

    start = False
    throttle_locked = False
    throttle_percent = 0
    throttle_velocity = 0
    arm_cards.clear()

    dpg.create_context()
    dpg.create_viewport(
        title="Solar Gamera",
        width=DEFAULT_VIEWPORT_WIDTH,
        height=DEFAULT_VIEWPORT_HEIGHT,
        min_height=850,
    )
    dpg.setup_dearpygui()

    fonts = create_fonts()
    arm_themes = create_arm_card_themes()

    with dpg.file_dialog(
        directory_selector=False,
        show=False,
        callback=save_file_callback,
        tag="save_file_dialog",
        width=700,
        height=400,
        default_filename="output.txt",
    ):
        dpg.add_file_extension(".txt")
        dpg.add_file_extension(".csv")

    with dpg.window(
        tag="main_window",
        no_scrollbar=True,
        no_scroll_with_mouse=True,
    ):
        # Header: a fixed title column and four equally stretchable arm cards.
        with dpg.table(
            tag="header_layout",
            header_row=False,
            policy=dpg.mvTable_SizingStretchSame,
            width=-1,
            no_pad_outerX=True,
        ):
            dpg.add_table_column(
                width_fixed=True,
                init_width_or_weight=TITLE_COLUMN_WIDTH,
            )
            for _ in ARM_SPECS:
                dpg.add_table_column(
                    width_stretch=True,
                    init_width_or_weight=1.0,
                )

            with dpg.table_row():
                with dpg.table_cell():
                    dpg.add_spacer(height=8)
                    dpg.add_text("Solar Gamera", tag="title_text")

                for arm_id, label, throttle, rpm in ARM_SPECS:
                    with dpg.table_cell() as card_cell:
                        card = ArmCard(
                            arm_id,
                            label,
                            throttle_percent=throttle,
                            rpm=rpm,
                            toggle_themes=arm_themes,
                            font=fonts["small"],
                        )
                        card.build(card_cell)
                        arm_cards[arm_id] = card

        # Body: corresponding sidebar and dashboard controls share table rows,
        # so their vertical alignment is handled directly by the layout.
        with dpg.table(
            tag="body_layout",
            header_row=False,
            policy=dpg.mvTable_SizingStretchProp,
            width=-1,
            no_pad_outerX=True,
        ):
            dpg.add_table_column(width_stretch=True, init_width_or_weight=1.0)
            dpg.add_table_column(width_stretch=True, init_width_or_weight=2.6)

            with dpg.table_row():
                with dpg.table_cell():
                    dpg.add_text(RSSI_val(), tag="RSSI_text")
                    dpg.add_text(LQ_val(), tag="LQ_text")
                    dpg.add_text(SNR_val(), tag="SNR_text")
                    with dpg.group(tag="control_button_stack"):
                        with dpg.table(
                            tag="throttle_limit_row",
                            header_row=False,
                            policy=dpg.mvTable_SizingStretchProp,
                            width=-1,
                            no_pad_outerX=True,
                        ):
                            dpg.add_table_column(
                                width_stretch=True,
                                init_width_or_weight=1.0,
                            )
                            dpg.add_table_column(
                                width_stretch=True,
                                init_width_or_weight=2.5,
                            )
                            dpg.add_table_column(
                                width_stretch=True,
                                init_width_or_weight=2.5,
                            )
                            with dpg.table_row():
                                with dpg.table_cell():
                                    dpg.add_button(
                                        label="UNLOCKED",
                                        tag="throttle_lock_button",
                                        callback=toggle_throttle_lock,
                                        width=-1,
                                        height=THROTTLE_CONTROL_ROW_HEIGHT,
                                    )
                                with dpg.table_cell():
                                    dpg.add_button(
                                        label="Throttle: Max",
                                        tag="max_throttle_button",
                                        callback=set_max_throttle,
                                        width=-1,
                                        height=THROTTLE_CONTROL_ROW_HEIGHT,
                                    )
                                with dpg.table_cell():
                                    dpg.add_button(
                                        label="Throttle: Min",
                                        tag="min_throttle_button",
                                        callback=set_min_throttle,
                                        width=-1,
                                        height=THROTTLE_CONTROL_ROW_HEIGHT,
                                    )

                        with dpg.table(
                            tag="velocity_control_row",
                            header_row=False,
                            policy=dpg.mvTable_SizingStretchProp,
                            width=-1,
                            no_pad_outerX=True,
                        ):
                            dpg.add_table_column(
                                width_stretch=True,
                                init_width_or_weight=1.0,
                            )
                            dpg.add_table_column(
                                width_stretch=True,
                                init_width_or_weight=4.0,
                            )
                            with dpg.table_row():
                                with dpg.table_cell():
                                    dpg.add_input_int(
                                        tag="throttle_velocity_input",
                                        default_value=0,
                                        min_value=0,
                                        min_clamped=True,
                                        step=0,
                                        step_fast=0,
                                        width=-1,
                                    )
                                with dpg.table_cell():
                                    dpg.add_button(
                                        label="Set Throttle Velocity\n(%/s, 0 = instant)",
                                        tag="set_velocity_button",
                                        callback=set_throttle_velocity,
                                        width=-1,
                                        height=THROTTLE_CONTROL_ROW_HEIGHT,
                                    )

                        with dpg.table(
                            tag="throttle_control_row",
                            header_row=False,
                            policy=dpg.mvTable_SizingStretchProp,
                            width=-1,
                            no_pad_outerX=True,
                        ):
                            dpg.add_table_column(
                                width_stretch=True,
                                init_width_or_weight=1.0,
                            )
                            dpg.add_table_column(
                                width_stretch=True,
                                init_width_or_weight=4.0,
                            )
                            with dpg.table_row():
                                with dpg.table_cell():
                                    dpg.add_input_int(
                                        tag="throttle_input",
                                        default_value=0,
                                        min_value=0,
                                        max_value=100,
                                        min_clamped=True,
                                        max_clamped=True,
                                        step=0,
                                        step_fast=0,
                                        width=-1,
                                    )
                                with dpg.table_cell():
                                    dpg.add_button(
                                        label="Set Throttle (%)",
                                        tag="set_throttle_button",
                                        callback=set_throttle,
                                        width=-1,
                                        height=THROTTLE_CONTROL_ROW_HEIGHT,
                                    )

                        with dpg.table(
                            tag="control_button_row",
                            header_row=False,
                            policy=dpg.mvTable_SizingStretchSame,
                            width=-1,
                            no_pad_outerX=True,
                        ):
                            dpg.add_table_column(width_stretch=True)
                            dpg.add_table_column(width_stretch=True)
                            with dpg.table_row():
                                with dpg.table_cell():
                                    dpg.add_button(
                                        label="Arm",
                                        tag="arm_button",
                                        callback=arm_button_fun,
                                        width=-1,
                                        height=ESTOP_RESET_ROW_HEIGHT,
                                    )
                                with dpg.table_cell():
                                    dpg.add_button(
                                        label="E-STOP Reset",
                                        tag="e_stop_button",
                                        callback=estop,
                                        width=-1,
                                        height=ESTOP_RESET_ROW_HEIGHT,
                                    )
                        dpg.add_button(
                            label="EMERGENCY STOP",
                            tag="stop_button",
                            callback=Emerstop,
                            width=-1,
                            height=ESTOP_ROW_HEIGHT,
                        )

                with dpg.table_cell():
                    with dpg.group(tag="graph_section"):
                        dpg.add_spacer(height=GRAPH_SECTION_TOP_MARGIN)
                        with dpg.table(
                            tag="plot_grid",
                            header_row=False,
                            policy=dpg.mvTable_SizingStretchSame,
                            width=-1,
                            no_pad_outerX=True,
                        ):
                            dpg.add_table_column(width_stretch=True)
                            dpg.add_table_column(width_stretch=True)

                            plot_specs = (
                                (("n", "N"), ("s", "S")),
                                (("e", "E"), ("w", "W")),
                            )
                            for plot_row in plot_specs:
                                with dpg.table_row():
                                    for arm_id, label in plot_row:
                                        with dpg.table_cell():
                                            add_arm_plot(arm_id, label)
                        dpg.add_spacer(height=GRAPH_SECTION_BOTTOM_MARGIN)

            with dpg.table_row():
                with dpg.table_cell():
                    dpg.add_button(
                        label="Start",
                        tag="start_button",
                        width=-1,
                        height=START_BUTTON_HEIGHT,
                        callback=start_button_fun,
                    )

                with dpg.table_cell():
                    dpg.add_button(
                        label="Choose Output File",
                        tag="choose_output_button",
                        width=-1,
                        height=OUTPUT_BUTTON_HEIGHT,
                        callback=open_save_dialog,
                    )

    apply_font("title_text", fonts["title"])
    apply_font("stop_button", fonts["estop"])
    apply_font("e_stop_button", fonts["medium"])
    apply_font("arm_button", fonts["medium"])
    apply_font("throttle_lock_button", fonts["lock"])
    apply_font("max_throttle_button", fonts["medium"])
    apply_font("min_throttle_button", fonts["medium"])
    apply_font("set_velocity_button", fonts["medium"])
    apply_font("set_throttle_button", fonts["medium"])
    apply_font("throttle_velocity_input", fonts["medium"])
    apply_font("throttle_input", fonts["medium"])
    apply_font("RSSI_text", fonts["medium"])
    apply_font("LQ_text", fonts["medium"])
    apply_font("SNR_text", fonts["medium"])
    apply_font("start_button", fonts["medium"])

    theme_main_window()
    theme_stop_button()
    dpg.set_primary_window("main_window", True)
    dpg.set_viewport_resize_callback(update_responsive_layout)

    dpg.show_viewport()
    dpg.set_frame_callback(1, update_responsive_layout)
    dpg.set_frame_callback(2, refresh_arm_card_layout)
    dpg.start_dearpygui()
    dpg.destroy_context()

# graph
pwm_data = []
rpm_data = []

def open_save_dialog(sender, app_data):
    dpg.show_item("save_file_dialog")

output_file_path = ""
throttle_locked = False
throttle_percent = 0
throttle_velocity = 0

def save_file_callback(sender, app_data):
    global output_file_path

    # app_data["file_path_name"] is the full selected path
    output_file_path = app_data["file_path_name"]
    print("Output file path:", output_file_path)


# Emergency Stop function
def Emerstop():
    print("EMERGENCY STOP pressed")

def estop():
    print("E-STOP pressed")

def arm_button_fun():
    print("Arm pressed")


def toggle_throttle_lock():
    """Enable or disable the Max and Min throttle shortcuts."""
    global throttle_locked
    throttle_locked = not throttle_locked
    dpg.configure_item(
        "throttle_lock_button",
        label="LOCKED" if throttle_locked else "UNLOCKED",
    )
    dpg.configure_item("max_throttle_button", enabled=not throttle_locked)
    dpg.configure_item("min_throttle_button", enabled=not throttle_locked)


def set_max_throttle():
    if throttle_locked:
        return
    set_throttle_value(100)


def set_min_throttle():
    if throttle_locked:
        return
    set_throttle_value(0)


def set_throttle_velocity():
    global throttle_velocity
    throttle_velocity = dpg.get_value("throttle_velocity_input")
    print(f"Throttle velocity set to {throttle_velocity}%/s")


def set_throttle():
    set_throttle_value(dpg.get_value("throttle_input"))


def set_throttle_value(value):
    global throttle_percent
    throttle_percent = max(0, min(100, int(value)))
    dpg.set_value("throttle_input", throttle_percent)
    print(f"Throttle set to {throttle_percent}%")

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

def start_button_fun():
    global start
    start = True
    print("Start pressed")


if __name__ == "__main__":
    main()
