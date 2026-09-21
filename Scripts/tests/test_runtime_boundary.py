from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class RuntimeBoundaryTest(unittest.TestCase):
    def fixture(self, source: str = '', link: str = '', runtimes: str = 'Runtime') -> tuple[Path, Path]:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        (root / 'Editor').mkdir()
        (root / 'Editor/Inspector.hpp').write_text('struct Inspector {};\n')
        (root / 'component.hpp').write_text('struct Component { float value = 0; };\n' + source)
        (root / 'Editor/CMakeLists.txt').write_text(
            'add_library(EditorTools INTERFACE)\n'
            'target_include_directories(EditorTools INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")\n'
            'add_library(Tools::Editor ALIAS EditorTools)\n')
        (root / 'CMakeLists.txt').write_text(f'''
cmake_minimum_required(VERSION 3.17)
project(BoundaryFixture NONE)
add_subdirectory(Editor)
add_library(Runtime INTERFACE)
target_sources(Runtime INTERFACE "${{CMAKE_CURRENT_SOURCE_DIR}}/component.hpp")
target_include_directories(Runtime INTERFACE "${{CMAKE_CURRENT_SOURCE_DIR}}")
{link}
include("{(ROOT / 'cmake/EvoEngineRuntimeBoundary.cmake').as_posix()}")
evoengine_enforce_runtime_boundary(ROOT "${{CMAKE_CURRENT_SOURCE_DIR}}"
  CHECKER "{(ROOT / 'Scripts/check_runtime_boundary.py').as_posix()}"
  TARGETS {runtimes} EDITOR_ROOTS "${{CMAKE_CURRENT_SOURCE_DIR}}/Editor")
''')
        build = root / 'build'
        result = subprocess.run(['cmake', '-S', str(root), '-B', str(build), '-DCMAKE_BUILD_TYPE=Debug'],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return root, build

    def run_check(self, build: Path, success: bool, message: str = '') -> None:
        result = subprocess.run(['cmake', '--build', str(build), '--config', 'Debug',
                                 '--target', 'EvoEngineRuntimeBoundary'], capture_output=True, text=True)
        self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
        self.assertIn(message, result.stdout + result.stderr)

    def test_plain_component_needs_no_gui_declaration(self):
        _, build = self.fixture('// #include "Inspector.hpp"\n')
        self.run_check(build, True)

    def test_guarded_relative_editor_include_fails(self):
        _, build = self.fixture('#if 0\n#include "Editor/Inspector.hpp"\n#endif\n')
        self.run_check(build, False, 'includes editor header')

    def test_transitive_header_cannot_hide_editor_include(self):
        root, build = self.fixture('#include "helper.hpp"\n')
        (root / 'helper.hpp').write_text('#include "Editor/Inspector.hpp"\n')
        self.run_check(build, False, 'helper.hpp includes editor header')

    def test_macro_include_cannot_bypass_header_check(self):
        _, build = self.fixture('#define HEADER "Editor/Inspector.hpp"\n#include HEADER\n')
        self.run_check(build, False, 'use a literal include path')

    def test_shared_header_checked_with_each_runtime_include_path(self):
        root, build = self.fixture('#include "helper.hpp"\n', link='''
target_include_directories(Runtime INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/safe")
add_library(OtherRuntime INTERFACE)
target_sources(OtherRuntime INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/component.hpp")
target_include_directories(OtherRuntime INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/unsafe")
''', runtimes='Runtime OtherRuntime')
        for directory in ('safe', 'unsafe'):
            (root / directory).mkdir()
        (root / 'safe/helper.hpp').write_text('struct Helper {};\n')
        (root / 'unsafe/helper.hpp').write_text('#include "../Editor/Inspector.hpp"\n')
        self.run_check(build, False, 'helper.hpp includes editor header')

    def test_configuration_specific_transitive_alias_link_fails(self):
        _, build = self.fixture(link='''
add_library(Bridge INTERFACE)
target_link_libraries(Bridge INTERFACE "$<$<CONFIG:Debug>:$<LINK_ONLY:Tools::Editor>>")
target_link_libraries(Runtime INTERFACE Bridge)
''')
        self.run_check(build, False, 'links editor target EditorTools')

    def test_core_imgui_is_allowed(self):
        _, build = self.fixture('void Gui() { ImGui::Text("value"); }')
        self.run_check(build, True)

    def test_gui_code_and_editor_guard_fail(self):
        for source in ('void Inspector() { ImGuizmo::BeginFrame(); }',
                       '#if EVOENGINE_WITH_EDITOR\nvoid Inspector();\n#endif'):
            with self.subTest(source=source):
                _, build = self.fixture(source)
                self.run_check(build, False, 'editor GUI code or per-class editor guard')

    def test_imported_interface_cannot_supply_editor_pch(self):
        _, build = self.fixture(link='''
add_library(ImportedHelper INTERFACE IMPORTED)
set_property(TARGET ImportedHelper PROPERTY INTERFACE_PRECOMPILE_HEADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/Editor/Inspector.hpp")
target_link_libraries(Runtime INTERFACE ImportedHelper)
''')
        self.run_check(build, False, 'editor source/PCH')


if __name__ == '__main__':
    unittest.main()
