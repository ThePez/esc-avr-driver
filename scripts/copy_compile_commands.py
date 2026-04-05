Import("env")
import os
import json

def generate_compile_commands(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    project_dir = env.subst("$PROJECT_DIR")
    
    flags = env.subst("$CCFLAGS $CFLAGS $_CCCOMCOM").split()
    defines = [env.subst(f) for f in env.get("CPPDEFINES", [])]

    # Collect all include paths - both from PlatformIO and our project dirs
    include_paths = set()
    
    # Get PlatformIO's include paths
    for path in env.get("CPPPATH", []):
        include_paths.add(env.subst(path))
    
    # Walk project dirs and add any directory containing headers
    search_dirs = [
        os.path.join(project_dir, "src"),
        os.path.join(project_dir, "lib"),
        os.path.join(project_dir, "include"),
    ]
    for search_dir in search_dirs:
        for root, dirs, files in os.walk(search_dir):
            if any(f.endswith(".h") for f in files):
                include_paths.add(root)

    include_flags = [f"-I{p}" for p in include_paths]

    commands = []
    for search_dir in search_dirs:
        for root, dirs, files in os.walk(search_dir):
            for f in files:
                if f.endswith(".c") or f.endswith(".cpp"):
                    filepath = os.path.join(root, f)
                    rel = os.path.relpath(filepath, project_dir)
                    obj = os.path.join(build_dir, rel.replace(".c", ".o").replace(".cpp", ".o"))

                    cmd = ["avr-gcc"]
                    cmd += flags
                    cmd += include_flags
                    cmd += [f"-D{d}" if isinstance(d, str) else f"-D{d[0]}={d[1]}" for d in defines]
                    cmd += ["-o", obj, filepath]

                    commands.append({
                        "directory": project_dir,
                        "command": " ".join(cmd),
                        "file": filepath
                    })

    out = os.path.join(project_dir, "compile_commands.json")
    with open(out, "w") as fp:
        json.dump(commands, fp, indent=2)
    print(f"Generated compile_commands.json with {len(commands)} entries")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", generate_compile_commands)