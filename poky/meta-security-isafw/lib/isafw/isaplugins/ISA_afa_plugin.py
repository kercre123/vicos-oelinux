#
# ISA_afa_plugin.py -  Anki - firmware analyzer plugin integration
#
import subprocess
import os
import sys
import copy
# try:
#     from lxml import etree
# except ImportError:
#     try:
#         import xml.etree.cElementTree as etree
#     except ImportError:
#         import xml.etree.ElementTree as etree

AFAChecker = None

class ISA_AFAChecker():
    initialized = False

    def __init__(self, ISA_config):
        self.full_reports = True
        self.logfile = ISA_config.logdir + "/isafw_afalog"
        self.full_report_name = ISA_config.reportdir + "/afa_full_report_" + \
            ISA_config.machine + "_" + ISA_config.timestamp
        tools_errors = _check_tools()
        if tools_errors:
            with open(self.logfile, 'w') as flog:
                flog.write(tools_errors)
                return
        self.initialized = True
        with open(self.logfile, 'a') as flog:
            flog.write("\nPlugin ISA_AFAChecker initialized!\n")
        return

    def process_filesystem(self, ISA_filesystem):
        self.ISA_filesystem = ISA_filesystem
        fs_path = self.ISA_filesystem.path_to_fs
        img_name = self.ISA_filesystem.img_name
        if (self.initialized):
            if (img_name and fs_path):
                with open(self.logfile, 'a') as flog:
                    flog.write("\n\nFilesystem path is: " + fs_path + "\n\n")
                if self.full_reports:
                    with open(self.full_report_name + "_" + img_name, 'w') as ffull_report:
                        ffull_report.write("AFAT result - image with rootfs location at " + fs_path + "\n\n")

                env = copy.deepcopy(os.environ)
		env['PSEUDO_UNLOAD'] = "1"
                try:
                    afat_path = subprocess.check_output(['which', 'afat.sh'], env=env).decode('utf-8')
                except:
                    with open(self.logfile, 'a') as flog:
                        flog.write("cannot find afat.sh in the PATH" + "\n\n")
                    return
                with open(self.logfile, 'a') as flog:
                    flog.write("\n\n" + afat_path + "\n\n")
                cmd = ['afat.sh', fs_path]

                try:
                    result = subprocess.check_output(cmd, env=env)
                except:
                    with open(self.logfile, 'a') as flog:
                        flog.write("error executing afat.sh \n")
                    return
                with open(self.full_report_name + "_" + img_name, 'a') as flog:
                    flog.write("\n\n" + result + "\n\n")
            else:
                with open(self.logfile, 'a') as flog:
                    flog.write(
                        "Mandatory argument, path to the filesystem are not provided!\n")
                    flog.write("Not performing the call.\n")
        else:
            with open(self.logfile, 'a') as flog:
                flog.write("Plugin hasn't initialized! Not performing the call.\n")

def _check_tools():

    def _is_in_path(executable):
        "Check for presence of executable in PATH"
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            if (os.path.isfile(os.path.join(path, executable)) and
                    os.access(os.path.join(path, executable), os.X_OK)):
                return True
        return False

    tools = {
        "afat.sh": "Please install afat.sh\n",
    }
    output = ""
    for tool in tools:
        if not _is_in_path(tool):
            output += tools[tool]
    return output

#======== supported callbacks from ISA =============#

def init(ISA_config):
    global AFAChecker 
    AFAChecker = ISA_AFAChecker(ISA_config)

def getPluginName():
    return "ISA_AFAChecker"

def process_filesystem(ISA_filesystem):
    global AFAChecker
    return AFAChecker.process_filesystem(ISA_filesystem)

# def process_report():
#     global AFAChecker
#     return AFAChecker.process_report()
#====================================================#
