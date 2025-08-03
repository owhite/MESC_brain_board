import os
import sys

# python3 replace_kicad_names.py data_logger VESC_brain_board

def rename_and_replace(target_dir, old_str, new_str):
    # First, rename files that include old_str in their names
    for root, dirs, files in os.walk(target_dir):
        for filename in files:
            old_path = os.path.join(root, filename)
            new_filename = filename.replace(old_str, new_str)
            new_path = os.path.join(root, new_filename)

            if old_path != new_path:
                os.rename(old_path, new_path)
                print(f"Renamed: {old_path} → {new_path}")

    # Now, replace text in file contents
    for root, dirs, files in os.walk(target_dir):
        for filename in files:
            file_path = os.path.join(root, filename)
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                new_content = content.replace(old_str, new_str)
                if new_content != content:
                    with open(file_path, 'w', encoding='utf-8') as f:
                        f.write(new_content)
                    print(f"Updated content: {file_path}")
            except UnicodeDecodeError:
                print(f"Skipped binary or non-text file: {file_path}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python rename_and_replace.py <old_name> <new_name>")
        sys.exit(1)

    old_name = sys.argv[1]
    new_name = sys.argv[2]
    current_directory = os.getcwd()

    rename_and_replace(current_directory, old_name, new_name)

