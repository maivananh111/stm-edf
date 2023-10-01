import sys
import os
import re
import logging
from PyQt5.QtWidgets import QMessageBox


HAL_project = "STM32Cube HAL Driver"
Base_project = "Base CMSIS Driver"


logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s')

# Hàm tạo hộp thông báo lỗi hoặc thông tin.
def make_message_box(level, title, label):
    message_box = QMessageBox()
    message_box.setIcon(level)  
    message_box.setWindowTitle(title)  
    message_box.setText(label)  
    message_box.exec_()


# Hàm lấy target name(vd: STM32F103CBTx, STM32F429IGTx,...)
# Trả về chuỗi chứa target name nếu thành công, None nếu không có file cproject hoặc không có target name(project lỗi).
def get_file_target_name(cproject_file_path):
    try:
        with open(cproject_file_path, 'r') as file:
            content = file.read()
            match = re.search(r'STM32[^"]*', content)
            if match:
                return match.group()
            else:
                return None
            
    except FileNotFoundError:
        logging.error(f"{cproject_file_path} -> No such file in directory.")
        return None
    
# Hàm lấy nền project 
# Trả về  HAL_project nếu được tạo từ HAL, Base_project nếu là project empty.
def get_project_background(cproject_file_path):
    try:
        with open(cproject_file_path, 'r') as file:
            content = file.read()
            match = content.find("HAL")
            if match != -1:
                return HAL_project
            else:
                return Base_project
            
    except FileNotFoundError:
        logging.error(f"{cproject_file_path} -> No such file in directory.")
        return "None"
    
# Hàm tạo file khôi phục trạng thái ban đầu của project (Có bao gồm kiểm tra sự tồn tại của file khôi phục trong folder)
def copy_file(src_file_path, dest_file_path, force = False):

    if (not os.path.isfile(dest_file_path) or force == True) and os.path.isfile(src_file_path):
        try:
            with open(src_file_path, 'rb') as src_file:
                src_content = src_file.read()

                with open(dest_file_path, 'wb') as dest_file:
                    dest_file.write(src_content)

                return True
            
        except FileNotFoundError:
            logging.error(f"{src_file_path} -> No such file in directory.")
            return  False
    
    else:
        return  True
    

# Hàm lấy đường dẫn EDF_PATH
def get_EDF_PATH():
    edf_path = os.environ.get('EDF_PATH')

    if edf_path == None:
        make_message_box(QMessageBox.Critical, "Error", "EDF_PATH Environment variable not found.")
    else:
        logging.info(f"EDF PATH: {edf_path}")
    
    return edf_path


def get_install_state(cproject_file_path):
    try:
        with open(cproject_file_path, 'r') as f:
            file_content = f.read()
        if file_content.find("STM_EDF_VERSION") != -1:
            return False
        else:
            return True
    # Hien message box neu khong mo duoc file.
    except FileNotFoundError:
        make_message_box(QMessageBox.Critical, "Error", f"{cproject_file_path}: No such file in directory.")
        logging.error(f"{cproject_file_path} -> No such file in directory.")
        return False
    

def get_project_name(project_file_path):
    try:
        with open(project_file_path, 'r') as f:
            file_content = f.read()

        pattern = r"<name>(.*?)</name>"
        match = re.search(pattern, file_content)
        if match:
            project_name = match.group(1)
            return project_name
        else:
            return None

    except FileNotFoundError:
        make_message_box(QMessageBox.Critical, "Error", f"{project_file_path}: No such file in directory.")
        logging.error(f"{project_file_path} -> No such file in directory.")
        return None
    

# Hàm tìm kiếm và thay thế một chuỗi theo patten.
def find_and_replace_printf_or_scanf(input_string, pattern, replace):
    # Search string in file match with pattern.
    match = re.search(pattern, input_string)
    if match: # Case 2 found string match with pattern.
        found_string = match.group(0)
        # Trường hợp 2: thay thế chuỗi option_string hiện có
        new_file_content = input_string.replace(found_string, replace, 1)
    else:
        # Trường hợp 1: chèn option_string vào trước chuỗi "<targetPlatform..."
        replace_index = '<targetPlatform archList="all" binaryParser="org.eclipse.cdt.core.ELF"'
        new_file_content = input_string.replace(replace_index, replace + "\n\t\t\t\t\t\t\t" + replace_index, 1)

    return new_file_content
