subdir('softfloat')

kvx_ss = ss.source_set()
kvx_ss.add(when: 'TARGET_KVX',
  if_true: files(
      'cpu.c',
      'cpu_helper.c',
      'debug_helper.c',
      'gdbstub.c',
      'lao-helper.c',
      'mds-helpers.c',
      'op_helper.c',
      'timers.c',
      'tlb_helper.c',
      'translate.c',
      'translate-insn.override.c',
      'gen/decode.c',
      'gen/translate-insn.c',
      'gen/mds-helpers.c'
  )
)

cc = meson.get_compiler('c')

lao_lib = cc.find_library('BSL', dirs: config_host['LAO_LIB_DIR'])
lao = declare_dependency(compile_args: config_host['LAO_CFLAGS'].split(),
                         link_args: '-Wl,-rpath=$ORIGIN/../lib/lao',
                         dependencies: lao_lib)

kvx_softfloat = declare_dependency(compile_args: '-I' + meson.current_source_dir() / 'softfloat' / 'source' / 'include',
                                   link_with: softfloat_lib)

kvx_ss.add(when: ['TARGET_KVX', 'CONFIG_LAO', lao, kvx_softfloat],
           if_true: files('lao-wrapper.c'))

kvx_softmmu_ss = ss.source_set()
kvx_softmmu_ss.add(files(
  'semihosting.c',
))

target_arch += {'kvx': kvx_ss}
target_softmmu_arch += {'kvx': kvx_softmmu_ss}
