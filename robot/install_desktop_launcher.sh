#!/usr/bin/env bash
# install_desktop_launcher.sh : run ONCE on the rover's computer. It puts two icons on the desktop:
#   "Run IMPRIMIS Rover"       starts the lap software (run_robot.sh)
#   "Check IMPRIMIS Rover"     only checks devices, settings and the build (run_robot.sh --check)
# Run it again whenever this folder is moved, because the icons hold its full path.
#
#   bash install_desktop_launcher.sh
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
DESKTOP="$(xdg-user-dir DESKTOP 2>/dev/null)"
[ -n "$DESKTOP" ] && [ -d "$DESKTOP" ] || DESKTOP="$HOME/Desktop"
mkdir -p "$DESKTOP" "$HOME/.local/share/applications"
chmod +x "$HERE/run_robot.sh" 2>/dev/null

make_icon() {      # file name, title, comment, extra argument
  local file="$1" title="$2" comment="$3" extra="$4"
  for where in "$DESKTOP" "$HOME/.local/share/applications"; do
    cat > "$where/$file" <<EOF
[Desktop Entry]
Type=Application
Version=1.0
Name=$title
Comment=$comment
Exec=bash "$HERE/run_robot.sh" $extra
Path=$HERE
Terminal=true
Icon=applications-engineering
Categories=Development;
EOF
    chmod +x "$where/$file"
    # GNOME shows a desktop icon as a plain file until it is marked as trusted
    gio set "$where/$file" metadata::trusted true 2>/dev/null
  done
  echo "made: $DESKTOP/$file"
}

make_icon "imprimis-rover-run.desktop" "Run IMPRIMIS Rover" "Start the IMPRIMIS lap software on the rover" ""
make_icon "imprimis-rover-check.desktop" "Check IMPRIMIS Rover" "Check devices, settings and the build; start nothing" "--check"

echo
echo "Two icons are on the desktop. Double-click one to use it."
echo "If an icon opens as text or shows a lock, right-click it and choose \"Allow Launching\"."
echo "They are also in the applications menu under the same names."
