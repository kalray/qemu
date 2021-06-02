softfloat_makefile = custom_target('softfloat-makefile',
                                   output: 'Makefile',
                                   input: 'build' / 'Linux-x86_64-GCC' / 'Makefile',
                                   command: ['sed', '-e', 's/-O2/& -fPIC/', '@INPUT@'],
                                   capture: true)

softfloat_platform_h = custom_target('softfloat-platform.h',
                                     output: 'platform.h',
                                     input: 'build' / 'Linux-x86_64-GCC' / 'platform.h',
                                     command: ['sed', '-e', '$a #define THREAD_LOCAL __thread', '@INPUT@'],
                                     capture: true)

softfloat_src_dir = meson.current_source_dir()
softfloat_build_dir = meson.current_build_dir()

softfloat_lib = custom_target('softfloat-lib',
                              output: 'softfloat.a',
                              input: [softfloat_makefile, softfloat_platform_h],
                              command: [ config_host['MAKE'], '-C', softfloat_build_dir, 'SOURCE_DIR=' + softfloat_src_dir / 'source'])
