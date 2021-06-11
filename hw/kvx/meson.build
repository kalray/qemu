kvx_ss = ss.source_set()

kvx_ss.add(fdt)
kvx_ss.add(files(
  'boot.c',
  'apic-gic.c',
  'apic-mailbox.c',
  'dsu-clock.c',
  'ipi-ctrl.c',
  'itgen.c',
  'mppa-cluster.c',
  'pwr-ctrl.c',
))

hw_arch += {'kvx': kvx_ss}
