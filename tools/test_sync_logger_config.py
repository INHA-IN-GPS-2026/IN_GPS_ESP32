from pathlib import Path
import unittest
from sync_logger_config import REQUIRED, CONFLICTING, settings, synchronize

ROOT = Path(__file__).resolve().parents[1]

class ConfigSyncTests(unittest.TestCase):
    def setUp(self):
        self.defaults = (ROOT/'sdkconfig.defaults').read_text(encoding='utf-8')
        # Legacy logger-enabled configuration, with an unrelated user setting.
        self.before = ('CONFIG_IDF_TARGET="esp32s3"\r\n'
                       'CONFIG_INGPS_BATCH_LOGGER=y\r\n'
                       'CONFIG_ULP_COPROC_RESERVE_MEM=4096\r\n'
                       'CONFIG_PARTITION_TABLE_SINGLE_APP=y\r\n'
                       '# CONFIG_PARTITION_TABLE_CUSTOM is not set\r\n'
                       'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"\r\n'
                       'CONFIG_PARTITION_TABLE_FILENAME="partitions_singleapp.csv"\r\n'
                       'CONFIG_BT_NIMBLE_MAX_CONNECTIONS=3\r\n'
                       '# user clock setting\r\nCONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=160\r\n')

    def test_legacy_memory_partition_and_connection_migrate_together(self):
        after = synchronize(self.before, self.defaults)
        parsed = settings(after)
        baseline = settings(self.defaults)
        for key in REQUIRED:
            self.assertEqual(parsed[key], baseline[key], key)
        for key in CONFLICTING:
            self.assertEqual(parsed[key], 'n', key)
        self.assertNotIn('CONFIG_PARTITION_TABLE_FILENAME', parsed)
        self.assertIn('# user clock setting\r\nCONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=160\r\n', after)

    def test_repeated_sync_preserves_every_byte(self):
        once = synchronize(self.before, self.defaults)
        self.assertEqual(synchronize(once, self.defaults), once)

    def test_other_target_rejected(self):
        with self.assertRaises(ValueError):
            synchronize(self.before.replace('"esp32s3"', '"esp32"'), self.defaults)

if __name__ == '__main__':
    unittest.main()
