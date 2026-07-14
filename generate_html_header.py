Import("env")
import os

src_dir = env.subst("$PROJECT_SRC_DIR")
html_path = os.path.join(src_dir, "radar_ui.html")
header_path = os.path.join(src_dir, "radar_web_ui.h")

# Only regenerate if HTML is newer than the header (or header is missing)
needs_update = not os.path.exists(header_path) or \
               os.path.getmtime(html_path) > os.path.getmtime(header_path)

if needs_update:
    with open(html_path, "r", encoding="utf-8") as f:
        html = f.read()

    with open(header_path, "w", encoding="utf-8") as f:
        f.write("/* Auto-generated from radar_ui.html — DO NOT EDIT */\n")
        f.write("#pragma once\n")
        f.write("#include <stddef.h>\n\n")
        f.write("static const char radar_ui_html[] =\n")
        for line in html.split("\n"):
            escaped = line.replace("\\", "\\\\").replace('"', '\\"')
            f.write('    "' + escaped + '\\n"\n')
        f.write(";\n\n")
        f.write("static const size_t radar_ui_html_len = sizeof(radar_ui_html) - 1;\n")

    print("[radar] Generated %s from %s" % (header_path, html_path))
else:
    print("[radar] %s is up-to-date" % header_path)
