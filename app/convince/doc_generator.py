import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

def generate_table(xml_folder):
    # Path to the folder where the script lives
    script_dir = os.path.dirname(os.path.abspath(__file__))
    readme_path = os.path.join(script_dir, "README.md")

    # read json file with module descriptions
    json_file = Path(__file__).parent / "modules_description.json"
    if json_file.exists():
        with open(json_file) as f:
            json_data = json.load(f)
    else:
        json_data = {}

    projects_desc = json_data.get("projects", {})
    applications_desc = json_data.get("applications", {})
    modules_desc = json_data.get("modules", {})

    with open(readme_path, "w") as f:
            f.write(f"# Documentation\n")
            f.write(f"{projects_desc.get('convince', '')}\n\n")

    for xml_file in Path(xml_folder).glob("*.xml"):
        # print(xml_file)
        tree = ET.parse(xml_file)
        root = tree.getroot()

        rows = []
        with open(readme_path, "a") as f:
            f.write(f"## Description for {xml_file.stem}\n")
            if applications_desc and xml_file.stem in applications_desc:
                f.write(f"{applications_desc[xml_file.stem]}\n\n")

        for module in root.findall("module"):
            name = module.find("name").text.strip()
            params = module.find("parameters")
            node = module.find("node")
            # print(name, params, node)
            desc = []
            if params is not None and params.text is not None:
                desc.append(f"{params.text.strip()}")
            else:
                desc.append("")
            if node is not None:
                desc.append(f"{node.text.strip()}")

            # add description from json data if available
            key = " ".join([name] + [desc[0]] if desc[0] else [name])
            # print(f"Looking for key: {key}.")
            if key in modules_desc:
                desc.append(modules_desc[key])
            else:
                desc.append("")

            rows.append(f"| `{name}` | {desc[0]} | {desc[1]} | {desc[2]} |")

        with open(readme_path, "a") as f:
            f.write("| Module | Parameters  | Node | Description |\n")
            # f.write("|--------|-------------|------|-------------|\n")
            f.write("| :- | :------ | :- | :----------- |\n")
            f.write("\n".join(rows))
            f.write("\n\n")

if __name__ == "__main__":
    generate_table("scripts")
