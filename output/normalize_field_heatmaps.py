#!/usr/bin/env python3
"""
Normalize SpreadsheetML heatmaps across multiple files.

The script scans all files in a folder (non-recursive), loads each SpreadsheetML workbook,
collects the two existing N x N numeric grids, computes one global min/max across all those
values, and rewrites their cell styles to heat_0..heat_100 using that shared range.

It also appends a third N x N grid (difference grid) to each workbook, where each cell is
abs(second_grid - first_grid) for the same row/column position. Difference colors are based
on global min/max difference across all files, with red = max difference and green = min.

Default folder: the script's own folder.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

SS_NS = "urn:schemas-microsoft-com:office:spreadsheet"
O_NS = "urn:schemas-microsoft-com:office:office"
X_NS = "urn:schemas-microsoft-com:office:excel"
HTML_NS = "http://www.w3.org/TR/REC-html40"

NS = {"ss": SS_NS}
STYLE_ID_ATTR = f"{{{SS_NS}}}StyleID"
TYPE_ATTR = f"{{{SS_NS}}}Type"
WORKBOOK_TAG = f"{{{SS_NS}}}Workbook"


@dataclass
class WorkbookData:
    path: Path
    tree: ET.ElementTree
    table: ET.Element
    grid_size: int
    first_grid: list[list[tuple[ET.Element, float]]]
    second_grid: list[list[tuple[ET.Element, float]]]
    second_grid_last_row_index: int
    difference_grid: list[list[float]] | None = None


def register_namespaces() -> None:
    ET.register_namespace("", SS_NS)
    ET.register_namespace("o", O_NS)
    ET.register_namespace("x", X_NS)
    ET.register_namespace("ss", SS_NS)
    ET.register_namespace("html", HTML_NS)


def is_spreadsheetml_workbook(root: ET.Element) -> bool:
    return root.tag == WORKBOOK_TAG


def parse_grid_size(table: ET.Element, rows: list[ET.Element]) -> int:
    expanded_column_count = table.get(f"{{{SS_NS}}}ExpandedColumnCount")
    if expanded_column_count:
        try:
            column_count = int(expanded_column_count)
            if column_count > 0:
                return column_count
        except ValueError:
            pass

    for row in rows:
        column_count = len(row.findall("ss:Cell", NS))
        if column_count > 0:
            return column_count

    return 0


def parse_numeric_cell(cell: ET.Element) -> float | None:
    data = cell.find("ss:Data", NS)
    if data is None:
        return None

    if data.get(TYPE_ATTR) != "Number":
        return None

    text = (data.text or "").strip()
    if not text:
        return None

    try:
        return float(text)
    except ValueError:
        return None


def parse_numeric_row(row: ET.Element, grid_size: int) -> list[tuple[ET.Element, float]] | None:
    cells = row.findall("ss:Cell", NS)
    if len(cells) < grid_size:
        return None

    parsed_cells: list[tuple[ET.Element, float]] = []
    for cell in cells[:grid_size]:
        value = parse_numeric_cell(cell)
        if value is None:
            return None
        parsed_cells.append((cell, value))

    return parsed_cells


def collect_two_grids(table: ET.Element) -> tuple[int, list[list[tuple[ET.Element, float]]], list[list[tuple[ET.Element, float]]], int] | None:
    rows = table.findall("ss:Row", NS)
    grid_size = parse_grid_size(table, rows)
    if grid_size <= 0:
        return None

    numeric_rows: list[tuple[int, list[tuple[ET.Element, float]]]] = []
    for index, row in enumerate(rows):
        parsed_row = parse_numeric_row(row, grid_size)
        if parsed_row is not None:
            numeric_rows.append((index, parsed_row))

    required_rows = grid_size * 2
    if len(numeric_rows) < required_rows:
        return None

    first_grid = [parsed_row for _, parsed_row in numeric_rows[:grid_size]]
    second_grid = [parsed_row for _, parsed_row in numeric_rows[grid_size:required_rows]]
    second_grid_last_row_index = numeric_rows[required_rows - 1][0]

    return grid_size, first_grid, second_grid, second_grid_last_row_index


def collect_value_range(workbooks: list[WorkbookData]) -> tuple[float, float, int]:
    min_value = math.inf
    max_value = -math.inf
    total_values = 0

    for workbook in workbooks:
        for grid in (workbook.first_grid, workbook.second_grid):
            for row in grid:
                for _, value in row:
                    total_values += 1
                    if value < min_value:
                        min_value = value
                    if value > max_value:
                        max_value = value

    return min_value, max_value, total_values


def build_difference_grids(workbooks: list[WorkbookData]) -> tuple[float, float, int]:
    min_difference = math.inf
    max_difference = -math.inf
    total_differences = 0

    for workbook in workbooks:
        difference_grid: list[list[float]] = []
        for row_index in range(workbook.grid_size):
            difference_row: list[float] = []
            for column_index in range(workbook.grid_size):
                first_value = workbook.first_grid[row_index][column_index][1]
                second_value = workbook.second_grid[row_index][column_index][1]
                difference = abs(second_value - first_value)
                difference_row.append(difference)

                total_differences += 1
                if difference < min_difference:
                    min_difference = difference
                if difference > max_difference:
                    max_difference = difference

            difference_grid.append(difference_row)

        workbook.difference_grid = difference_grid

    return min_difference, max_difference, total_differences


def style_id_from_ratio(ratio: float) -> str:
    index = int(round(min(max(ratio, 0.0), 1.0) * 100.0))
    index = min(max(index, 0), 100)
    return f"heat_{index}"


def style_id_for_difference(value: float, min_value: float, max_value: float) -> str:
    if max_value - min_value <= 1e-12:
        return "heat_100"

    ratio = normalize(value, min_value, max_value)
    # Invert for difference grid: red means largest difference, green means smallest.
    return style_id_from_ratio(1.0 - ratio)


def apply_value_styles(workbook: WorkbookData, min_value: float, max_value: float) -> int:
    updated_cells = 0

    for grid in (workbook.first_grid, workbook.second_grid):
        for row in grid:
            for cell, value in row:
                cell.set(STYLE_ID_ATTR, style_id_for_value(value, min_value, max_value))
                updated_cells += 1

    return updated_cells


def trim_rows_after_second_grid(workbook: WorkbookData) -> None:
    rows = workbook.table.findall("ss:Row", NS)
    trailing_rows = rows[workbook.second_grid_last_row_index + 1 :]
    for row in trailing_rows:
        workbook.table.remove(row)


def create_separator_row() -> ET.Element:
    return ET.Element(f"{{{SS_NS}}}Row", {f"{{{SS_NS}}}Height": "12"})


def format_number(value: float) -> str:
    return f"{value:.6f}"


def append_difference_grid(workbook: WorkbookData, min_difference: float, max_difference: float) -> int:
    if workbook.difference_grid is None:
        return 0

    trim_rows_after_second_grid(workbook)
    workbook.table.append(create_separator_row())

    appended_cells = 0
    for difference_row in workbook.difference_grid:
        row_element = ET.Element(f"{{{SS_NS}}}Row")

        for difference in difference_row:
            cell_style = style_id_for_difference(difference, min_difference, max_difference)
            cell_element = ET.Element(f"{{{SS_NS}}}Cell", {STYLE_ID_ATTR: cell_style})
            data_element = ET.Element(f"{{{SS_NS}}}Data", {TYPE_ATTR: "Number"})
            data_element.text = format_number(difference)
            cell_element.append(data_element)
            row_element.append(cell_element)
            appended_cells += 1

        workbook.table.append(row_element)

    row_count = len(workbook.table.findall("ss:Row", NS))
    workbook.table.set(f"{{{SS_NS}}}ExpandedRowCount", str(row_count))
    return appended_cells


def collect_workbook_data(tree: ET.ElementTree) -> tuple[ET.Element, int, list[list[tuple[ET.Element, float]]], list[list[tuple[ET.Element, float]]], int] | None:
    root = tree.getroot()
    table = root.find(".//ss:Table", NS)
    if table is None:
        return None

    grids = collect_two_grids(table)
    if grids is None:
        return None

    grid_size, first_grid, second_grid, second_grid_last_row_index = grids
    return table, grid_size, first_grid, second_grid, second_grid_last_row_index


def load_workbooks(folder: Path) -> list[WorkbookData]:
    workbooks: list[WorkbookData] = []

    for path in sorted(folder.iterdir()):
        if not path.is_file():
            continue

        try:
            tree = ET.parse(path)
        except ET.ParseError:
            continue

        root = tree.getroot()
        if not is_spreadsheetml_workbook(root):
            continue

        workbook_data = collect_workbook_data(tree)
        if workbook_data is None:
            continue

        table, grid_size, first_grid, second_grid, second_grid_last_row_index = workbook_data

        workbooks.append(
            WorkbookData(
                path=path,
                tree=tree,
                table=table,
                grid_size=grid_size,
                first_grid=first_grid,
                second_grid=second_grid,
                second_grid_last_row_index=second_grid_last_row_index,
            )
        )

    return workbooks


def normalize(value: float, min_value: float, max_value: float) -> float:
    delta = max_value - min_value
    if delta <= 1e-12:
        return 0.5
    ratio = (value - min_value) / delta
    return min(max(ratio, 0.0), 1.0)


def style_id_for_value(value: float, min_value: float, max_value: float) -> str:
    ratio = normalize(value, min_value, max_value)
    return style_id_from_ratio(ratio)


def write_workbook(path: Path, root: ET.Element) -> None:
    xml_body = ET.tostring(root, encoding="unicode")
    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write("<?xml version=\"1.0\"?>\n")
        f.write("<?mso-application progid=\"Excel.Sheet\"?>\n")
        f.write(xml_body)
        if not xml_body.endswith("\n"):
            f.write("\n")


def update_workbooks(
    workbooks: list[WorkbookData],
    value_min: float,
    value_max: float,
    difference_min: float,
    difference_max: float,
    dry_run: bool,
) -> tuple[int, int]:
    updated_existing_cells = 0
    appended_difference_cells = 0

    for workbook in workbooks:
        updated_existing_cells += apply_value_styles(workbook, value_min, value_max)
        appended_difference_cells += append_difference_grid(workbook, difference_min, difference_max)

        if not dry_run:
            write_workbook(workbook.path, workbook.tree.getroot())

    return updated_existing_cells, appended_difference_cells


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Recompute heatmap styles using one global min/max across all SpreadsheetML files in a folder."
    )
    parser.add_argument(
        "folder",
        nargs="?",
        default=str(Path(__file__).resolve().parent),
        help="Folder to scan (default: script folder).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute and report only; do not rewrite files.",
    )
    args = parser.parse_args()

    folder = Path(args.folder).resolve()
    if not folder.exists() or not folder.is_dir():
        print(f"[error] Folder does not exist or is not a directory: {folder}")
        return 1

    register_namespaces()
    workbooks = load_workbooks(folder)
    if not workbooks:
        print(f"[info] No compatible SpreadsheetML files with two numeric N x N grids found in: {folder}")
        return 0

    value_min, value_max, total_values = collect_value_range(workbooks)
    difference_min, difference_max, total_differences = build_difference_grids(workbooks)
    updated_existing_cells, appended_difference_cells = update_workbooks(
        workbooks,
        value_min,
        value_max,
        difference_min,
        difference_max,
        args.dry_run,
    )

    mode = "dry-run" if args.dry_run else "updated"
    print(
        f"[{mode}] files={len(workbooks)} "
        f"value_cells={total_values} value_min={value_min:.9g} value_max={value_max:.9g} "
        f"difference_cells={total_differences} difference_min={difference_min:.9g} difference_max={difference_max:.9g} "
        f"updated_cells={updated_existing_cells} appended_difference_cells={appended_difference_cells}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
