env = Environment()
env.Replace(CXX = 'clang++')
env.Append(CPPFLAGS = ['-O2', '-std=c++1z', '-g'])

rootdir = './'
directories = []

files = env.Glob(rootdir + '*.cpp')
for item in directories:
    files += env.Glob(rootdir + item + '/*.cpp')

noclean = env.Program('shizuku', files)
env.NoClean(noclean)
