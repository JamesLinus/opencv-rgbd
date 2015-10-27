-- http://industriousone.com/scripting-reference
-- https://github.com/premake/premake-core/wiki

local action = _ACTION or ""

solution "opencv-rgbd"
    location ("_project")
    configurations { "Debug", "Release" }
    platforms {"x64", "x32"}
    language "C++"
    targetdir ("bin")
    kind "ConsoleApp"

    configuration "vs*"
        defines { "_CRT_SECURE_NO_WARNINGS" }
        if not os.isfile("bin/opencv_world300d.dll") then
            os.copyfile("../opencv-lib/vs2013-x86/opencv_world300d.dll", "bin/opencv_world300d.dll")
        end
        if not os.isfile("bin/opencv_world300.dll") then
            os.copyfile("../opencv-lib/vs2013-x86/opencv_world300.dll", "bin/opencv_world300.dll")
        end

    flags {
        "MultiProcessorCompile"
    }

    libdirs {
        "../opencv-lib/vs2013-x86",
        "bin",
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
        targetdir ("bin")
        defines { "DEBUG" }
        flags { "Symbols"}
        targetsuffix "-d"

    configuration "Release"
        defines { "NDEBUG" }
        flags { "Optimize"}

    project "opencv-rgbd"
        kind "StaticLib"
        includedirs {
            "include",
            "src",
            "../opencv-lib/include",
            "../opencv-lib/include/opencv2",
        }

        files {
            "include/*",
            "src/*",
        }

        defines {

        }

    project "linemod"
        includedirs {
            "include",
            "../opencv-lib/include",
        }
        files {
            "samples/linemod.cpp"
        }
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
            "../opencv-lib/include",
        }
        files {
            "samples/linemod.cpp"
        }
        configuration "Debug"
            links {
                "opencv-rgbd-d.lib"
            }
        configuration "Release"
            links {
                "opencv-rgbd.lib"
            }

