-- http://industriousone.com/scripting-reference
-- https://github.com/premake/premake-core/wiki

local action = _ACTION or ""

solution "opencv-rgbd"
    location ("_project")
    configurations { "Debug", "Release" }
    platforms {"x64", "x86"}
    language "C++"
    kind "ConsoleApp"

    openni_path = nil

    configuration "vs*"
        defines { "_CRT_SECURE_NO_WARNINGS" }

        configuration "x86"
            openni_path = "C:/Program Files (x86)/OpenNI2"
            libdirs {
                "../opencv-lib/vs2013-x86",
                "x86",
            }
            targetdir ("x86")

        configuration "x64"
            openni_path = "C:/Program Files/OpenNI2"
            libdirs {
                "../opencv-lib/vs2013-x64",
                "x64",
            }
            targetdir ("x64")

        os.mkdir("x86");
        os.copyfile("../opencv-lib/vs2013-x86/opencv_world300d.dll", "x86/opencv_world300d.dll")
        os.copyfile("../opencv-lib/vs2013-x86/opencv_world300.dll", "x86/opencv_world300.dll")
        os.mkdir("x64");
        os.copyfile("../opencv-lib/vs2013-x64/opencv_world300d.dll", "x64/opencv_world300d.dll")
        os.copyfile("../opencv-lib/vs2013-x64/opencv_world300.dll", "x64/opencv_world300.dll")

    flags {
        "MultiProcessorCompile"
    }

    configuration "Debug"
        links {
            "opencv_world300d.lib"
        }

    configuration "Release"
        links {
            "opencv_world300.lib"
        }

    configuration "Debug"
        defines { "DEBUG" }
        flags { "Symbols"}
        targetsuffix "-d"

    configuration "Release"
        defines { "NDEBUG" }
        flags { "Optimize"}

-- 
    project "opencv-rgbd"
        kind "StaticLib"

        includedirs {
            "include",
            "src",
            "../opencv-lib/include",
            path.join(openni_path, "include"),
        }

        files {
            "include/opencv2/*",
            "src/*",
        }

        defines {

        }

    project "linemod"
        includedirs {
            "include",
            "src",
            "../opencv-lib/include",
            path.join(openni_path, "include"),
        }
        libdirs {
            path.join(openni_path, "lib"),
        }
        
        files {
            "samples/linemod.cpp"
        }

        links "OpenNI2.lib"
        configuration "Debug"
            links {
                "opencv-rgbd-d.lib"
            }
        configuration "Release"
            links {
                "opencv-rgbd.lib"
            }

    project "odometry_evaluation"
        includedirs {
            "include",
            "src",
            "../opencv-lib/include",
            path.join(openni_path, "include"),
        }
        libdirs {
            path.join(openni_path, "lib"),
        }
        files {
            "samples/odometry_evaluation.cpp"
        }
        links "OpenNI2.lib"
        configuration "Debug"
            links {
                "opencv-rgbd-d.lib"
            }
        configuration "Release"
            links {
                "opencv-rgbd.lib"
            }

