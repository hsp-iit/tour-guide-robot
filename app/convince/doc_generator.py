import xml.etree.ElementTree as ET
from pathlib import Path

def generate_table(xml_folder, md_file):
    with open(md_file, "w") as f:
            f.write(f"# Documentation\n\n")
    for xml_file in Path(xml_folder).glob("*.xml"):
        # print(xml_file)
        tree = ET.parse(xml_file)
        root = tree.getroot()

        rows = []
        with open(md_file, "a") as f:
            f.write(f"## Documentation for {xml_file.stem}\n\n")

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

            rows.append(f"| `{name}` | {desc[0]} | {desc[1]} |")

        with open(md_file, "a") as f:
            f.write("| Module | Parameters  | Node |\n")
            f.write("|--------|-------------|------|\n")
            f.write("\n".join(rows))

if __name__ == "__main__":
    generate_table("scripts", "README2.md")
