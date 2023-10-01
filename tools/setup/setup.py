import sys
import os
import re
import logging
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QComboBox, QPushButton, QLabel, QFileDialog, QMainWindow, QMessageBox, QCheckBox
from gui import Ui_MainWindow
from function import *

project_file         = ".project"
cproject_file        = ".cproject"
recovery_file_sufix  = "_old"

HAL_project          = "STM32Cube HAL Driver"
Base_project         = "Base CMSIS Driver"
EDF_PATH_VAR         = "EDF_PATH"

device_cortex_series = {
    'STM32F0': 'M0',
    'STM32C0': 'M0+',
    'STM32L0': 'M0+',
    'STM32G0': 'M0+',
    'STM32F1': 'M3',
    'STM32F2': 'M3',
    'STM32L1': 'M3',
    'STM32F3': 'M4',
    'STM32F4': 'M4',
    'STM32G4': 'M4',
    'STM32L4': 'M4',
    'STM32F7': 'M7',
    'STM32h7': 'M7',
}
    

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s')



#----------------------------------------------------------------------------------------------------------------------------------------------------------------
class installer_app(Ui_MainWindow, QWidget):
    def __init__(self, window, app):
        self._window = window
        self._app = app
        super().__init__()
        self.setupUi(self._window)
        logging.info("Setup tools statting up.")
        
    # Add widget event handler.
        self.Btn_Browser.clicked.connect(self.onButtonBrowserClicked)
        self.Btn_Setup.clicked.connect(self.onButtonSetupClicked)
        self.Btn_Restore.clicked.connect(self.onButtonRestoreClicked)
        self.Btn_Quit.clicked.connect(self.onButtonQuitClicked)
        self.Enable_Widget(False)
        
    # Get EDF_PATH environment variable, exit if EDF_PATH is not define.
        self._edf_path = get_EDF_PATH()
        if self._edf_path == None: 
            sys.exit()

    # Check argument.
        if len(sys.argv) > 1:
            arg_project_dir = sys.argv[1]
            if arg_project_dir:
                self.onButtonBrowserClicked(arg_project_dir)

    # Show installer application.
        self._window.show()
        sys.exit(self._app.exec_())

    # Enable/disable widget function.
    def Enable_Widget(self, en):
        self.Btn_Setup.setDisabled(not en)
        self.Btn_Restore.setDisabled(not en)
        self.Box_Optimize.setDisabled(not en)
        self.CB_Printf_Float.setDisabled(not en)
        self.CB_Scanf_Float.setDisabled(not en)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# Quit button clickec handler.
    def onButtonQuitClicked(self):
        sys.exit()

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# Browser button clicked handler.
    def onButtonBrowserClicked(self, arg):
        # Get project directory in line edit.
        if arg:
            self._project_dir = arg
        else:
            self._project_dir = QFileDialog.getExistingDirectory(self, "Project browser", os.path.expanduser("~"), QFileDialog.ShowDirsOnly)

        if self._project_dir:
            self.LE_Project_Dir.setText(self._project_dir) # Set show directory.

            self._project_file_dir = self._project_dir + "/" + project_file
            self._cproject_file_dir = self._project_dir + "/" + cproject_file

        # Get project background.
            self._project_bgr = get_project_background(self._cproject_file_dir)
            self.Btn_Project_Bgr.setText(self._project_bgr)

        # Get project name.
            self._project_name = get_project_name(self._project_file_dir)
            if self._project_name != None:
            # Get .ioc file name.
                if self._project_bgr == HAL_project:
                    self._ioc_file_dir = self._project_dir + "/" + self._project_name + ".ioc"
            else:
                logging.error("Can't get project name.")
                make_message_box(QMessageBox.Critical, "Error", "Can't get project name.")
                return 

        # Get device full name in .cproject file.
            self._device_full_name = get_file_target_name(self._cproject_file_dir)
        # Get device series.
            self._device_family_series = self._device_full_name[:7]
        # Get device cortex series.
            self._device_cortex_series = device_cortex_series[self._device_family_series]
        # Get project install state.
            if get_install_state(self._cproject_file_dir) != True:
                self.Btn_Setup.setDisabled(True)
            else:
                self.Btn_Setup.setDisabled(False)
        
            if self._device_full_name == None:
                self.Btn_Device_Full_Name.setText("STM32xxxxxxxxx")
                self.Btn_Device_Series.setText("Unknown")
                make_message_box(QMessageBox.Critical, "Error", "Unknown STM32 device name.")
                logging.error("Unknown device name in project.")
                self.Enable_Widget(False)
                return
            else:
                self.Btn_Device_Full_Name.setText(self._device_full_name)
                self.Btn_Device_Series.setText(self._device_full_name[:7] + f'(Cortex-{self._device_cortex_series})')
                self.Enable_Widget(True)



#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# Setup button handler.    
    def onButtonSetupClicked(self):
        logging.info(f"Project Information:")
        logging.info(f"\tDirectory: {self._project_dir}")
        logging.info(f"\tName:      {self._project_name}")
        logging.info(f"\tDevice:    {self._device_full_name}")
        logging.info(f"\tSeries:    {self._device_family_series}")
        logging.info(f"\Core:       {self._device_cortex_series}")
    
    # Create recovery file.
        copy_file(self._cproject_file_dir, self._cproject_file_dir + recovery_file_sufix, False)
        copy_file(self._project_file_dir,  self._project_file_dir  + recovery_file_sufix, False)
        if self._project_bgr == HAL_project:
            copy_file(self._ioc_file_dir,  self._ioc_file_dir + recovery_file_sufix,      False)

    # Get build option optimize.
        sel_opt = self.Box_Optimize.currentText()
        start_index = sel_opt.find("(") + 1
        end_index = sel_opt.find(")", start_index)
        self.build_optimize_level = sel_opt[start_index:end_index]
        logging.info(f"\tOptimize level: {self.build_optimize_level}")
    # Get option printf float.
        if self.CB_Printf_Float.checkState() == 0:
            self.printf_float = "false"
        else:
            self.printf_float = "true"
        logging.info(f"\tPrintf float: {self.printf_float}")
    # Get option scanf float.
        if self.CB_Scanf_Float.checkState() == 0:
            self.scanf_float = "false"
        else:
            self.scanf_float = "true"
        logging.info(f"\tScanf float: {self.scanf_float}")


# Config .cproject file.
        c_symbols_replace   = "<listOptionValue builtIn=\"false\" value=\"DEBUG\"/>\n"
        symbols_insert = (    "\n\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"DEBUG\"/>\n"
                                "\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"_GNU_SOURCE\"/>\n"
                                "\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"__STM32__\"/>\n"
                               f"\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"{self._device_family_series}\"/>\n"
                               f"\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"{self._device_full_name}\"/>\n"
                               f"\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"DEVICE_NAME=&quot;{self._device_full_name}&quot;\"/>")
        symbols_insert_HAL = ("\n\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"USE_HAL_DRIVER\"/>")

        source_folder_replace = "<sourceEntries>"
        source_folder_insert = ("\t\t\t\t\t\t<entry flags=\"VALUE_WORKSPACE_PATH|RESOLVED\" kind=\"sourcePath\" name=\"edf_core\"/>\n"
                                "\t\t\t\t\t\t<entry flags=\"VALUE_WORKSPACE_PATH|RESOLVED\" kind=\"sourcePath\" name=\"edf_rtos\"/>\n"
                                "\t\t\t\t\t\t<entry flags=\"VALUE_WORKSPACE_PATH|RESOLVED\" kind=\"sourcePath\" name=\"main\"/>")

        c_include_path_replace   = "name=\"Include paths (-I)\" superClass=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.option.includepaths\" useByScannerDiscovery=\"false\" valueType=\"includePath\">"
        cpp_include_path_replace = "name=\"Include paths (-I)\" superClass=\"com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.cpp.compiler.option.includepaths\" useByScannerDiscovery=\"false\" valueType=\"includePath\">"
        include_path_insert = ( "\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"../buildconfig\"/>\n"
                               f"\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"&quot;${{{EDF_PATH_VAR}}}/components/core/include&quot;\"/>\n"
                               f"\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"&quot;${{{EDF_PATH_VAR}}}/components/freertos/os_c{self._device_cortex_series.lower()}&quot;\"/>\n")
        
        c_optimize_level_replace   =  'name="Optimization level" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.option.optimization.level" useByScannerDiscovery="false"/>'
        cpp_optimize_level_replace =  'name="Optimization level" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.cpp.compiler.option.optimization.level" useByScannerDiscovery="false"/>'
        c_optimize_level_pattern   = r'name="Optimization level" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.option.optimization.level" useByScannerDiscovery="false" value="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.tool\.c\.compiler\.option\.optimization\.level\.value\..*?\" valueType="enumerated"/>'
        cpp_optimize_level_pattern = r'name="Optimization level" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.cpp.compiler.option.optimization.level" useByScannerDiscovery="false" value="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.tool\.cpp\.compiler\.option\.optimization\.level\.value\..*?\" valueType="enumerated"/>'
        c_optimize_level_insert    = f'name="Optimization level" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.option.optimization.level" useByScannerDiscovery="false" value="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.c.compiler.option.optimization.level.value.{self.build_optimize_level}\" valueType="enumerated"/>'
        cpp_optimize_level_insert  = f'name="Optimization level" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.cpp.compiler.option.optimization.level" useByScannerDiscovery="false" value="com.st.stm32cube.ide.mcu.gnu.managedbuild.tool.cpp.compiler.option.optimization.level.value.{self.build_optimize_level}\" valueType="enumerated"/>'
        
        printf_float_pattern = r'<option id="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.option\.nanoprintffloat\.\d+" name="Use float with printf from newlib-nano \(-u _printf_float\)" superClass="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.option\.nanoprintffloat" useByScannerDiscovery="false" value=".+" valueType="boolean"/>'
        printf_float_replace = f'<option id="com.st.stm32cube.ide.mcu.gnu.managedbuild.option.nanoprintffloat.1816458521" name="Use float with printf from newlib-nano (-u _printf_float)" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.option.nanoprintffloat" useByScannerDiscovery="false" value="{self.printf_float}" valueType="boolean"/>'
        
        scanf_float_pattern =  r'<option id="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.option\.nanoscanffloat\.\d+" superClass="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.option\.nanoscanffloat" value=".+" valueType="boolean"/>'
        scanf_float_replace =  f'<option id="com.st.stm32cube.ide.mcu.gnu.managedbuild.option.nanoscanffloat.653624109" superClass="com.st.stm32cube.ide.mcu.gnu.managedbuild.option.nanoscanffloat" value="{self.scanf_float}" valueType="boolean"/>'


        try:
            with open(self._cproject_file_dir, 'r') as f:
                file_content = f.read()
                new_file_content = file_content
                # Xóa hết các symbols hiện có.
                # pattern = re.compile(c_symbols_pattern, re.DOTALL)
                # new_file_content = re.sub(pattern, r'\1\n\t\t\t\t\t\t\t\t\2', file_content)
                # pattern = re.compile(cpp_symbols_pattern, re.DOTALL)
                # new_file_content = re.sub(pattern, r'\1\n\t\t\t\t\t\t\t\t\2', new_file_content)
                # Insert symbols
                if self._project_bgr == HAL_project:
                    new_file_content = new_file_content.replace(c_symbols_replace, c_symbols_replace + symbols_insert + symbols_insert_HAL)
                    # new_file_content = new_file_content.replace(cpp_symbols_replace, cpp_symbols_replace + symbols_insert + symbols_insert_HAL)
                else:
                    new_file_content = new_file_content.replace(c_symbols_replace, c_symbols_replace + symbols_insert)
                    # new_file_content = new_file_content.replace(cpp_symbols_replace, cpp_symbols_replace + symbols_insert)
                # Insert c include path
                new_file_content = new_file_content.replace(c_include_path_replace, c_include_path_replace + "\n" + include_path_insert)
                # Insert cpp include path
                new_file_content = new_file_content.replace(cpp_include_path_replace, cpp_include_path_replace + "\n" + include_path_insert)
                # Insert source folder directory
                new_file_content = new_file_content.replace(source_folder_replace, source_folder_replace + "\n" + source_folder_insert)
                # Insert c optimize level
                if new_file_content.find(c_optimize_level_replace) != -1:
                    new_file_content = new_file_content.replace(c_optimize_level_replace, c_optimize_level_insert)
                else:
                    new_file_content = re.sub(c_optimize_level_pattern, c_optimize_level_insert, new_file_content, count=1)
                # Insert cpp optimize level
                if new_file_content.find(cpp_optimize_level_replace) != -1:
                    new_file_content = new_file_content.replace(cpp_optimize_level_replace, cpp_optimize_level_insert)
                else:
                    new_file_content = re.sub(cpp_optimize_level_pattern, cpp_optimize_level_insert, new_file_content, count=1)

                # Change printf float option.
                new_file_content = find_and_replace_printf_or_scanf(new_file_content, printf_float_pattern, printf_float_replace)
                # Change scanf float option.
                new_file_content = find_and_replace_printf_or_scanf(new_file_content, scanf_float_pattern, scanf_float_replace)

                with open(self._cproject_file_dir, 'w') as f:
                    f.write(new_file_content)
                logging.info(f"Config .cproject file successful")
        
        except FileNotFoundError:
            make_message_box(QMessageBox.Critical, "Error", f"{self._cproject_file_dir}: No such file in directory.")
            logging.error(f"{self._cproject_file_dir} -> No such file in directory.")
            return
        

                
# Config .project file.
        project_desciption_replace = "</projectDescription>"
        project_desciption_insert = ("\t<linkedResources>\n"
                                        "\t\t<link>\n"
                                        "\t\t\t<name>edf_core</name>\n"
                                        "\t\t\t<type>2</type>\n"
                                       f"\t\t\t<location>{self._edf_path}/components/core/source</location>\n"
                                        "\t\t</link>\n"
                                        "\t\t<link>\n"
                                        "\t\t\t<name>edf_rtos</name>\n"
                                        "\t\t\t<type>2</type>\n"
                                       f"\t\t\t<location>{self._edf_path}/components/freertos/os_c{self._device_cortex_series.lower()}</location>\n"
                                        "\t\t</link>\n"
                                        "\t</linkedResources>\n"
                                        "</projectDescription>")
        
        linked_source_check = "<name>edf_core</name>"

        try:
            with open(self._project_file_dir, 'r') as f:
                file_content = f.read()

            if not (linked_source_check in file_content):
                new_file_content = file_content.replace(project_desciption_replace, project_desciption_insert)
                with open(self._project_file_dir, 'w') as file:
                    file.write(new_file_content)

            logging.info(f"Config .project file successful")

        except FileNotFoundError:
            make_message_box(QMessageBox.Critical, "Error", f"{self._project_file_dir}: No such file in directory.")
            logging.error(f"{self._project_file_dir} -> No such file in directory.")
            return
        


# Config .ioc file.
        if self._project_bgr == HAL_project:
            with open(self._ioc_file_dir, 'r') as f:
                file_content = f.read()

            busfault_handler_replace   = "NVIC.BusFault_IRQn=true\:0\:0\:false\:false\:true\:false\:false\:false"
            debugmon_handler_replace   = "NVIC.DebugMonitor_IRQn=true\:0\:0\:false\:false\:true\:false\:false\:false"
            hardfault_handler_replace  = "NVIC.HardFault_IRQn=true\:0\:0\:false\:false\:true\:false\:false\:false"
            memmanage_handler_replace  = "NVIC.MemoryManagement_IRQn=true\:0\:0\:false\:false\:true\:false\:false\:false"
            nmi_handler_replace        = "NVIC.NonMaskableInt_IRQn=true\:0\:0\:false\:false\:true\:false\:false\:false"
            pendsv_handler_replace     = "NVIC.PendSV_IRQn=true\:0\:0\:false\:false\:true\:false\:false\:false"
            svcall_handler_replace     = "NVIC.SVCall_IRQn=true\:0\:0\:false\:false\:true\:false\:false\:false"
            systick_handler_replace    = "NVIC.SysTick_IRQn=true\:15\:0\:false\:false\:true\:false\:true\:false"
            usagefault_handler_replace = "NVIC.UsageFault_IRQn=true\:0\:0\:false\:false\:true\:false\:false\:false"

            busfault_handler_insert   = "NVIC.BusFault_IRQn=true\:0\:0\:false\:false\:false\:false\:false\:false"
            debugmon_handler_insert   = "NVIC.DebugMonitor_IRQn=true\:0\:0\:false\:false\:false\:false\:false\:false"
            hardfault_handler_insert  = "NVIC.HardFault_IRQn=true\:0\:0\:false\:false\:false\:false\:false\:false"
            memmanage_handler_insert  = "NVIC.MemoryManagement_IRQn=true\:0\:0\:false\:false\:false\:false\:false\:false"
            nmi_handler_insert        = "NVIC.NonMaskableInt_IRQn=true\:0\:0\:false\:false\:false\:false\:false\:false"
            pendsv_handler_insert     = "NVIC.PendSV_IRQn=true\:0\:0\:false\:false\:false\:false\:false\:false"
            svcall_handler_insert     = "NVIC.SVCall_IRQn=true\:0\:0\:false\:false\:false\:false\:false\:false"
            systick_handler_insert    = "NVIC.SysTick_IRQn=true\:15\:0\:false\:false\:false\:false\:false\:false"
            usagefault_handler_insert = "NVIC.UsageFault_IRQn=true\:0\:0\:false\:false\:false\:false\:false\:false"
            
            
            if busfault_handler_replace in file_content:
                new_file_content = file_content.replace(busfault_handler_replace, busfault_handler_insert)
            if debugmon_handler_replace in file_content:
                new_file_content = new_file_content.replace(debugmon_handler_replace, debugmon_handler_insert)
            if hardfault_handler_replace in file_content:
                new_file_content = new_file_content.replace(hardfault_handler_replace, hardfault_handler_insert)
            if memmanage_handler_replace in file_content:
                new_file_content = new_file_content.replace(memmanage_handler_replace, memmanage_handler_insert)
            if nmi_handler_replace in file_content:
                new_file_content = new_file_content.replace(nmi_handler_replace, nmi_handler_insert)
            if pendsv_handler_replace in file_content:
                new_file_content = new_file_content.replace(pendsv_handler_replace, pendsv_handler_insert)
            if svcall_handler_replace in file_content:
                new_file_content = new_file_content.replace(svcall_handler_replace, svcall_handler_insert)
            if systick_handler_replace in file_content:
                new_file_content = new_file_content.replace(systick_handler_replace, systick_handler_insert)
            if usagefault_handler_replace in file_content:
                new_file_content = new_file_content.replace(usagefault_handler_replace, usagefault_handler_insert)

            with open(self._ioc_file_dir, 'w') as f:
                f.write(new_file_content)
            logging.info(f"Config .ioc file successful")




# Add edf_main_application into file main.c.
        if self._project_bgr == HAL_project:
            main_file_path = os.path.join(self._project_dir, "Core/Src/main.c")
            try:
                with open(main_file_path, 'r') as f:
                    file_content = f.read()

                # Lấy các hàm init của STM32 HAL Driver.
                pattern = re.compile(r'/\*\s*Initialize all configured peripherals\s*\*/\n\s*(.*?)\n\s*/\*\s*USER CODE BEGIN 2\s*\*/', re.DOTALL)
                match = pattern.search(file_content)
                if match:
                    HAL_init_func = match.group(1)
                    if file_content.find("/* USER CODE BEGIN 0 */") != -1:
                        new_file_content = file_content.replace("/* USER CODE BEGIN 0 */", 
                                                                "/* USER CODE BEGIN 0 */\n"
                                                                "void HAL_driver_init(void){\n"
                                                               f"\t{HAL_init_func}"
                                                                "\n}" 
                                                                )
                if file_content.find("main_application") == -1:
                    if file_content.find("/* USER CODE BEGIN 1 */") != -1:
                        new_file_content = new_file_content.replace("/* USER CODE BEGIN 1 */", 
                                                                "/* USER CODE BEGIN 1 */\n"
                                                                "\textern int edf_main_application(void);\n"
                                                                "\treturn edf_main_application();"
                                                                )
                        
                with open(main_file_path, 'w') as file:
                    file.write(new_file_content)

            # Hien message box neu khong mo duoc file.
            except FileNotFoundError:
                make_message_box(QMessageBox.Critical, "Error", "main.c: No such file in directory.")
                logging.error(f"/Core/main.c -> No such file in directory.")
                return
        
# Remove exception_interrupt_handler from file stm32fxxx_it.c.
        if self._project_bgr == HAL_project:
            it_file = "Core/Src/" + self._device_family_series.lower() + "xx_it.c"
            it_file_path = os.path.join(self._project_dir, it_file)

            start_marker = ("/**\n"
                            "  * @brief This function handles Non maskable interrupt.\n"
                            "  */\n"
                            "void NMI_Handler(void)")
            end_marker = "/* USER CODE END SysTick_IRQn 1 */\n}"

            try:
                with open(it_file_path, "r") as input_file:
                    content = input_file.read()

                start_index = content.find(start_marker)
                end_index = content.find(end_marker, start_index)

                if start_index != -1 and end_index != -1:
                    output_content = content[:start_index] + content[end_index + len(end_marker):]
                    
                    with open(it_file_path, "w") as output_file:
                        output_file.write(output_content)
                else:
                    logging.error(f"{it_file_path}: Error during edit file.")

            except FileNotFoundError:
                make_message_box(QMessageBox.Critical, "Error", f"{it_file_path}: No such file in directory.")
                logging.error(f"{it_file_path}: No such file in directory.")
                return
            
# Create folder main.
        try:
            os.system(f"mkdir {self._project_dir}/main")
            os.system(f"cp {self._edf_path}/components/templates/source/app_main.cpp {self._project_dir}/main/")
        # Create folder config.
            os.system(f"mkdir {self._project_dir}/buildconfig")
            os.system(f"cp {self._edf_path}/components/templates/header/* {self._project_dir}/buildconfig")
            os.system(f"cp {self._edf_path}/components/templates/kconfig/* {self._project_dir}/buildconfig")
        except Exception as e:
            logging.error(f"Error creating folder: {e}")

        # Get project state.
        try:
            with open(self._cproject_file_dir, 'r') as f:
                file_content = f.read()
            if file_content.find("STM_EDF_VERSION") != -1:
                self.Btn_Setup.setDisabled(True)
            else:
                self.Btn_Setup.setDisabled(False)

        except FileNotFoundError:
            make_message_box(QMessageBox.Critical, "Error", ".cproject: No such file in directory.")
            logging.error(f"{self._cproject_file_dir} -> No such file in directory.")
            return
        
        make_message_box(QMessageBox.Information, "Progress", "Setup successful.")
        logging.info("Setup successful.")


#----------------------------------------------------------------------------------------------------------------------------------------------------------------
# Restore button handler.
    def onButtonRestoreClicked(self):
        logging.info(f"Uninstall STM32 RTOSSDK form {self._project_dir}")

# Trả về cấu hình ban đầu cho .cproject và .project.
        if (not copy_file(self._cproject_file_dir + recovery_file_sufix, self._cproject_file_dir, True)) or \
            not copy_file(self._project_file_dir + recovery_file_sufix, self._project_file_dir, True) or \
            not copy_file(self._ioc_file_dir + recovery_file_sufix, self._ioc_file_dir, True) :
            make_message_box(QMessageBox.Critical, "Error", "Can't Restore from project.")
            logging.error("Can't uninstall from project.")

# Remove edf function main.c and stm32xxxxx_it.c
        if self._project_bgr == HAL_project:
    # main.c
            main_file_path = os.path.join(self._project_dir, "Core/Src/main.c")
            try:
                with open(main_file_path, 'r') as f:
                    file_content = f.read()

                    pattern = re.compile(r'\/\*\s*USER CODE BEGIN 0\s*\*\/\n.*?\n\s*\/\*\s*USER CODE END 0\s*\*\/', re.DOTALL)
                    main_c_replace = '/* USER CODE BEGIN 0 */\n\n/* USER CODE END 0 */'
                    new_file_content = re.sub(pattern, main_c_replace, file_content)

                    pattern = re.compile(r'\/\*\s*USER CODE BEGIN 1\s*\*\/\n.*?\n\s*\/\*\s*USER CODE END 1\s*\*\/', re.DOTALL)
                    main_c_replace = '/* USER CODE BEGIN 1 */\n\n  /* USER CODE END 1 */'
                    new_file_content = re.sub(pattern, main_c_replace, new_file_content)

                with open(main_file_path, 'w') as file:
                    file.write(new_file_content)

            # Hien message box neu khong mo duoc file.
            except FileNotFoundError:
                make_message_box(QMessageBox.Critical, "Error", "main.c: No such file in directory.")
                logging.error("main.c: No such file in directory.")
                return
    

        # Get project state.
        try:
            with open(self._cproject_file_dir, 'r') as f:
                file_content = f.read()
            if file_content.find("STM_EDF_VERSION") != -1:
                self.Btn_Setup.setDisabled(True)
            else:
                self.Btn_Setup.setDisabled(False)

        except FileNotFoundError:
            make_message_box(QMessageBox.Critical, "Error", ".cproject: No such file in directory.")
            
        make_message_box(QMessageBox.Information, "Progress", "Restore successful.")
        print("Uninstall successful.")


        
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = QMainWindow()
    inst_gui = installer_app(window, app)










	# extern int main_application(void);
	# return main_application();

	# <linkedResources>
	# 	<link>
	# 		<name>rtossdk</name>
	# 		<type>2</type>
	# 		<location>/home/anh/Projects/CODE/STM32/RTOSSDK/rtossdk</location>
	# 	</link>
	# </linkedResources>



#         if self.projectbgr == HAL_project:
#             it_file = "Core/Src/" + self.device_series.lower() + "xx_it.c";
#             it_file_path = os.path.join(self.projectdir, it_file)
#             try:
#                 with open(it_file_path, 'r') as f:
#                     file_content = f.read()

#             # Include libary and declare variable
#                 if file_content.find("/* USER CODE BEGIN Includes */") != -1:
#                     new_file_content = file_content.replace("/* USER CODE BEGIN Includes */", 
#                                                             "/* USER CODE BEGIN Includes */\n"
#                                                             "#include \"freertos_port/app_port/freertos_port.h\""
#                                                             )
#                 if file_content.find("/* USER CODE BEGIN EV */") != -1:
#                     new_file_content = new_file_content.replace("/* USER CODE BEGIN EV */", 
#                                                             "/* USER CODE BEGIN EV */\n"
#                                                             "extern void exception_interrupt_handler(const char *tag, char *message);\n"
#                                                             "static const char *Excep_TAG = \"EXCEPTION\";"
#                                                             )
#             # Fault notify.
#                 if file_content.find("/* USER CODE BEGIN HardFault_IRQn 0 */") != -1:
#                     new_file_content = new_file_content.replace("/* USER CODE BEGIN HardFault_IRQn 0 */", 
#                                                             "/* USER CODE BEGIN HardFault_IRQn 0 */\n"
#                                                             "\texception_interrupt_handler(Excep_TAG, (char *)\"Hard fault exception was unhandled(call HardFault_Handler)...\");"
#                                                             )
#                 if file_content.find("/* USER CODE BEGIN MemoryManagement_IRQn 0 */") != -1:
#                     new_file_content = new_file_content.replace("/* USER CODE BEGIN MemoryManagement_IRQn 0 */", 
#                                                             "/* USER CODE BEGIN MemoryManagement_IRQn 0 */\n"
#                                                             "\texception_interrupt_handler(Excep_TAG, (char *)\"Memory management interrupt was unhandled(call MemManage_Handler)...\");"
#                                                             )
#                 if file_content.find("/* USER CODE BEGIN BusFault_IRQn 0 */") != -1:
#                     new_file_content = new_file_content.replace("/* USER CODE BEGIN BusFault_IRQn 0 */", 
#                                                             "/* USER CODE BEGIN BusFault_IRQn 0 */\n"
#                                                             "\texception_interrupt_handler(Excep_TAG, (char *)\"Bus fault exception was unhandled(call BusFault_Handler)...\");"
#                                                             )
#                 if file_content.find("/* USER CODE BEGIN UsageFault_IRQn 0 */") != -1:
#                     new_file_content = new_file_content.replace("/* USER CODE BEGIN UsageFault_IRQn 0 */", 
#                                                             "/* USER CODE BEGIN UsageFault_IRQn 0 */\n"
#                                                             "\texception_interrupt_handler(Excep_TAG, (char *)\"Usage fault exception was unhandled(call UsageFault_Handler)...\");"
#                                                             )
#             # Port freertos handler.
#                 if file_content.find("/* USER CODE BEGIN SVCall_IRQn 0 */") != -1:
#                     new_file_content = new_file_content.replace("/* USER CODE BEGIN SVCall_IRQn 0 */", 
#                                                             "/* USER CODE BEGIN SVCall_IRQn 0 */\n"
#                                                             "\tfreertos_svc_handler();"
#                                                             )
#                 if file_content.find("/* USER CODE BEGIN PendSV_IRQn 0 */") != -1:
#                     new_file_content = new_file_content.replace("/* USER CODE BEGIN PendSV_IRQn 0 */", 
#                                                             "/* USER CODE BEGIN PendSV_IRQn 0 */\n"
#                                                             "\tfreertos_pendsv_handler();"
#                                                             )
#                 if file_content.find("/* USER CODE BEGIN SysTick_IRQn 0 */") != -1:
#                     new_file_content = new_file_content.replace("/* USER CODE BEGIN SysTick_IRQn 0 */", 
#                                                             "/* USER CODE BEGIN SysTick_IRQn 0 */\n"
#                                                             "\textern void systick_app_systick_process(void);\n"
#                                                             "\tsystick_app_systick_process();\n"
#                                                             "\tfreertos_tick_handler();"
#                                                             )
                
#                 with open(it_file_path, 'w') as file:
#                     file.write(new_file_content)

#             # Hien message box neu khong mo duoc file.
#             except FileNotFoundError:
#                 make_message_box(QMessageBox.Critical, "Error", f"{it_file}: No such file in directory.")
#                 return


    # # stm32xxxxx_it.c
    #         it_file = "Core/Src/" + self.device_series.lower() + "xx_it.c";
    #         it_file_path = os.path.join(self.projectdir, it_file)
    #         try:
    #             with open(it_file_path, 'r') as f:
    #                 file_content = f.read()

    #                 pattern = re.compile(r'\/\*\s*USER CODE BEGIN EV\s*\*\/\n.*?\n\s*\/\*\s*USER CODE END EV\s*\*\/', re.DOTALL)
    #                 main_c_replace = '/* USER CODE BEGIN EV */\n\n  /* USER CODE END EV */'
    #                 new_file_content = re.sub(pattern, main_c_replace, file_content)

    #                 pattern = re.compile(r'\/\*\s*USER CODE BEGIN HardFault_IRQn 0\s*\*\/\n.*?\n\s*\/\*\s*USER CODE END HardFault_IRQn 0\s*\*\/', re.DOTALL)
    #                 main_c_replace = '/* USER CODE BEGIN HardFault_IRQn 0 */\n\n  /* USER CODE END HardFault_IRQn 0 */'
    #                 new_file_content = re.sub(pattern, main_c_replace, new_file_content)

    #                 pattern = re.compile(r'\/\*\s*USER CODE BEGIN MemoryManagement_IRQn 0\s*\*\/\n.*?\n\s*\/\*\s*USER CODE END MemoryManagement_IRQn 0\s*\*\/', re.DOTALL)
    #                 main_c_replace = '/* USER CODE BEGIN MemoryManagement_IRQn 0 */\n\n  /* USER CODE END MemoryManagement_IRQn 0 */'
    #                 new_file_content = re.sub(pattern, main_c_replace, new_file_content)

    #                 pattern = re.compile(r'\/\*\s*USER CODE BEGIN BusFault_IRQn 0\s*\*\/\n.*?\n\s*\/\*\s*USER CODE END BusFault_IRQn 0\s*\*\/', re.DOTALL)
    #                 main_c_replace = '/* USER CODE BEGIN BusFault_IRQn 0 */\n\n  /* USER CODE END BusFault_IRQn 0 */'
    #                 new_file_content = re.sub(pattern, main_c_replace, new_file_content)

    #                 pattern = re.compile(r'\/\*\s*USER CODE BEGIN UsageFault_IRQn 0\s*\*\/\n.*?\n\s*\/\*\s*USER CODE END UsageFault_IRQn 0\s*\*\/', re.DOTALL)
    #                 main_c_replace = '/* USER CODE BEGIN UsageFault_IRQn 0 */\n\n  /* USER CODE END UsageFault_IRQn 0 */'
    #                 new_file_content = re.sub(pattern, main_c_replace, new_file_content)

    #                 pattern = re.compile(r'\/\*\s*USER CODE BEGIN SysTick_IRQn 0\s*\*\/\n.*?\n\s*\/\*\s*USER CODE END SysTick_IRQn 0\s*\*\/', re.DOTALL)
    #                 main_c_replace = '/* USER CODE BEGIN SysTick_IRQn 0 */\n\n  /* USER CODE END SysTick_IRQn 0 */'
    #                 new_file_content = re.sub(pattern, main_c_replace, new_file_content)

    #             with open(it_file_path, 'w') as file:
    #                 file.write(new_file_content)

    #         # Hien message box neu khong mo duoc file.
    #         except FileNotFoundError:
    #             make_message_box(QMessageBox.Critical, "Error", "main.c: No such file in directory.")
    #             return




