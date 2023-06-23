import os
import re
import shutil

Import("env")

resSrcDir = os.path.join(env.subst("$BUILD_DIR"), "resources", "src")
resBuildDir = os.path.join(env.subst("$BUILD_DIR"), "resources", "build")
resDir = os.path.join(env.subst("$PROJECT_DIR"), "resources")

print("Resource Dir: ", resDir)

if os.path.exists(resBuildDir):
    shutil.rmtree(resBuildDir)
os.makedirs(resBuildDir)

if os.path.exists(resSrcDir):
    shutil.rmtree(resSrcDir)
os.makedirs(resSrcDir)

def genResource(inputFile):
    with open(inputFile, "rb") as f:
        data = f.read()
    fileSize = len(data)
    
    inputBase = os.path.basename(inputFile)
    funcName = "GetResource_" + re.sub(r"[^a-zA-Z0-9]", "_", inputBase)

    outputFile = os.path.join(resSrcDir, inputBase) + ".cpp"

    with open(outputFile, "wt") as f:
        print("#include <stddef.h>\n#include <string_view>\nextern \"C\" {\nstatic const unsigned char contents[] = { ", file=f, end='')
        print(", ".join("0x%02x" % x for x in data), file=f, end='')
        print(" };", file=f)
        print("const unsigned char* {}{}(size_t* len) {{\n  *len = {};\n  return contents;\n}}\n}}".format("", funcName, fileSize), file=f)

        print("std::string_view {}() {{\n  return std::string_view(reinterpret_cast<const char*>(contents), {});\n}}".format(funcName, fileSize), file=f)



# Loop through everything in the resources folder
# For each item, generate the resource cpp file
for filename in os.listdir(resDir):
    f = os.path.join(resDir, filename)
    if os.path.isfile(f):
        genResource(f)

# ensure that we add the built stuff to the build path
env.BuildSources(
    os.path.join("$BUILD_DIR", "resources", "build"),
    os.path.join("$BUILD_DIR", "resources", "src")
)

