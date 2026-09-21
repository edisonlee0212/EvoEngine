#pragma once
#include <imgui.h>
#include <string>
#include <vector>
#include "EvoEngineEditorAPI.hpp"

namespace ImGui {
/**
 * @brief Creates a splitter widget for resizing UI panels.
 * @param split_vertically Indicates whether the splitter is vertical or horizontal.
 * @param thickness The thickness of the splitter.
 * @param size1 Reference to the size of the first panel.
 * @param size2 Reference to the size of the second panel.
 * @param min_size1 Minimum size of the first panel.
 * @param min_size2 Minimum size of the second panel.
 * @param splitter_long_axis_size Length of the splitter along the long axis. Defaults to -1.0f for auto-calculation.
 * @return True if the sizes of the panels were modified, false otherwise.
 */
EVOENGINE_EDITOR_API bool Splitter(bool split_vertically, float thickness, float& size1, float& size2, float min_size1,
                                   float min_size2, float splitter_long_axis_size = -1.0f);

/**
 * @brief Creates a combo box widget with a list of selectable items.
 * @param label The label for the combo box widget.
 * @param items A list of selectable items.
 * @param current_selection Reference to an unsigned value storing the current selection.
 * @param flags Optional flags for customizing the combo box behavior. Defaults to 0.
 * @return True if the selection was changed, false otherwise.
 */
EVOENGINE_EDITOR_API bool Combo(const std::string& label, const std::vector<std::string>& items,
                                unsigned& current_selection, ImGuiComboFlags flags = 0);

/**
 * @brief Creates a combo box widget with a list of selectable items.
 * @param label The label for the combo box widget.
 * @param items A list of selectable items.
 * @param current_selection Reference to an int value storing the current selection.
 * @param flags Optional flags for customizing the combo box behavior. Defaults to 0.
 * @return True if the selection was changed, false otherwise.
 */
EVOENGINE_EDITOR_API bool Combo(const std::string& label, const std::vector<std::string>& items, int& current_selection,
                                ImGuiComboFlags flags = 0);
}  // namespace ImGui
