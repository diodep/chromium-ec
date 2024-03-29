#!/usr/bin/env python
# Copyright (c) 2014 The Chromium OS Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
"""Flash PD PSU RW firmware over the USBPD comm channel using console."""

import array
import errno
import hashlib
import logging
import optparse
import os
import re
import socket
import sys
import time

import serial
# TODO(tbroch): Discuss adding hdctools as an EC package RDEPENDS
from servo import client
from servo import multiservo

VERSION = '0.0.1'

# RW area is half of the 32-kB flash minus the hash storage area
MAX_FW_SIZE = 16 * 1024 - 32
# Hash of RW when erased (set to all F's)
ERASED_RW_HASH = 'd582e94d 0d12a61c 1199927e 5610c036 2e2870a9'


class FlashPDError(Exception):
  """Exception class for flash_pd utility."""


class FlashPD(client.ServoClient):
  """class to flash PD MCU.

  Note,
  Some designs(samus) have multiple embedded MCUs.  In that case the convention
  is to name the pty associated with usbpd as 'usbpd_uart_pty'.  In the case
  where there is only one MCU (fruitpie) we prefer 'usbpd_uart_pty' but will
  also associate 'ecu_uart_pty' with having capability to flash the UBS-PD
  capable PSU(zinger).

  Attributes:
    _options : Values instance from optparse.

  Public Methods:
    expect        :  Examine console output for an expected response.
    flash_command :  Write a PD flash command and interrogate its result.
    get_version   :  Retrieve current version of PD FW.
  """

  def __init__(self, options):
    """Constructor.

    Args:
      options : Values instance from optparse.

    Raises:
      FlashPDError: If unable to determine the console pty
    """
    super(FlashPD, self).__init__(host=options.server, port=options.port)
    self._options = options
    self._serial = None

    try:
      pty = self.get('usbpd_uart_pty')
    except socket.error as e:
      raise FlashPDError('Can\'t connect to servod :: %s' % e)
    except client.ServoClientError:
      pty = self.get('ec_uart_pty')
    if not pty:
      raise FlashPDError('Unable to determine EC uart from servod')

    logging.debug('Opening serial connection to %s', pty)
    try:
      self._serial = serial.Serial(pty, timeout=1)
    except OSError as e:
      if e.errno == errno.EAGAIN:
        # try twice if already open EAGAIN failure causes disconnect.
        self._serial = serial.Serial(pty, timeout=1)
      else:
        raise FlashPDError('%s' % e)

    # quiet other channels that might pollute console.
    self._serial.write('chan 1\n')
    self._serial.flushOutput()
    self._serial.flushInput()

  def __del__(self):
    """Deconstructor."""
    if self._serial:
      for l in self._serial:
        logging.debug('flash: %s', l)
      self._serial.write('chan 0xffffffff\n')
      self._serial.write('chan restore\n')
      self._serial.close()

  def expect(self, val, timeout=5):
    """Scan serial output for particular string.

    Args:
      val     : string to look for
      timeout : integer seconds to look before timing out.

    Returns:
      tuple : boolean if 'val' found in console output.
              string of line that had 'val' in it.
    """
    done = False
    deadline = time.time() + timeout
    while not done and (time.time() < deadline):
      l = None
      for l in self._serial:
        done = val in l
        logging.debug('Is %s in: %s', val, l)
        if done or time.time() > deadline:
          break
    if not done:
      logging.error('\"%s\" missing', val)
    return (done, l)

  def flash_command(self, cmd, expect='DONE'):
    """Send PD Flash command and interrogate output.

    Args:
      cmd    : string of 'pd port flash' command to execute
      expect : string of expected response after 'cmd'

    Returns:
      found : boolean, whether response matches expected.
    """
    self._serial.write('pd %d flash %s\n' % (self._options.multiport, cmd))
    (found, line) = self.expect(expect)
    return (found, line)

  def get_version(self):
    """Retreive PSU firmware version.

    Looks like: 'version: zinger_v1.1.1917-bfd'

    Returns:
      version : string of version
    Raises:
      FlashPDError : if can't determine version
    """
    (found, line) = self.flash_command('version', expect='version:')
    logging.debug('is version in: %s', line)
    m = False
    if found:
      m = re.match(r'.*version:\s+(\w+_v\d+\.\d+\.\d+-\S+).*', line)
    if not m:
      raise FlashPDError('Unable to determine PD FW version')
    return m.group(1)


def flash_pd(options):
  """Flash power delivery firmware."""

  ec = FlashPD(options)

  with open(options.firmware) as fd:
    fw = fd.read()
    fw_size = len(fw)
    # Compute SHA-1 hash for the full (padded) RW firmware
    padded_fw = fw + '\xff' * (MAX_FW_SIZE - fw_size)
    sha = hashlib.sha1(padded_fw).digest()
    sha_str = ' '.join(['%08x' % (w) for w in array.array('I', sha)])

    # pad the firmware to a multiple of 6 U32
    if fw_size % 24:
      fw += '\xff'*(24 - fw_size % 24)
    words = array.array('I', fw)

  logging.info('Current PD FW version is %s', ec.get_version())
  if options.versiononly:
    return

  logging.info('Flashing %d bytes', fw_size)

  # reset flashed hash to reboot in RO
  ec.flash_command('hash' + ' 00000000' * 5)
  # reboot in RO
  ec.flash_command('reboot')
  # delay to give time to reboot
  time.sleep(1.5)
  # erase all RW partition
  ec.flash_command('erase')

  # verify that erase was successful by reading hash of RW
  (done, _) = ec.flash_command('rw_hash', expect=ERASED_RW_HASH)
  if done:
    done = ec.expect('DONE')

  if not done:
    raise FlashPDError('Erase failed')

  logging.info('Successfully erased flash.')

  if options.eraseonly:
    ec.flash_command('reboot')
    logging.info('After erase, FW version is %s', ec.get_version())
    return

  # write firmware content
  for i in xrange(len(words) / 6):
    chunk = words[i * 6: (i + 1) * 6]
    cmd = ' '.join(['%08x' % (w) for w in chunk])
    ec.flash_command(cmd)

  # write new firmware hash
  ec.flash_command('hash ' + sha_str)
  # reboot in RW
  ec.flash_command('reboot')
  # delay for reboot
  time.sleep(1.5)

  logging.info('Flashing DONE.')
  logging.info('SHA-1: %s', sha_str)
  logging.info('New PD FW version is %s', ec.get_version())


def parse_args():
  """Parse commandline arguments.

  Note, reads sys.argv directly

  Returns:
    options : dict of from optparse.parse_args().

  Raises:
    FlashPDError : If problems with arguments
  """
  description = (
      '%prog [<switch args>] <firmware.bin>'
      '\n'
      '%prog is a utility for flashing the USB-PD charger RW firmware over '
      'the USB-PD communication channel using PD MCU console commands.'
      )
  examples = (
      '\nExamples:\n'
      '   %prog build/zinger/ec.RW.flat\n'
      )
  parser = optparse.OptionParser(version='%prog ' + VERSION)
  parser.description = description
  parser.add_option('-d', '--debug', action='store_true', default=False,
                    help='enable debug messages.')
  parser.add_option('-s', '--server', help='host where servod is running',
                    default=client.DEFAULT_HOST)
  parser.add_option('-p', '--port', default=client.DEFAULT_PORT, type=int,
                    help='port servod is listening on.')
  parser.add_option('-m', '--multiport', default=0, type=int,
                    help='If design has multiple type-C ports, this identifies '
                    'which one has USB PD PSU.')
  parser.add_option('', '--timeout', default=5, type=int,
                    help='Timeout seconds to wait for console output.')
  parser.add_option('', '--eraseonly', action='store_true', default=False,
                    help='Only erase RW portion and exit.')
  parser.add_option('-V', '--versiononly', action='store_true', default=False,
                    help='Only read version and exit.')
  multiservo.add_multiservo_parser_options(parser)

  parser.set_usage(parser.get_usage() + examples)
  (options, args) = parser.parse_args()

  # TODO(tbroch) Add this once we refactor module to ease use in scripts.
  if options.name:
    raise NotImplementedError('Multiservo support TBD')

  # Add after to enumerate options.firmware but outside 'help' generation
  parser.add_option('-f', '', action='store', type='string', dest='firmware')

  if len(args) != 1:
    raise FlashPDError('Must supply power delivery firmware to write.')

  options.firmware = args[0]
  if not os.path.exists(options.firmware):
    raise FlashPDError('Unable to find file %s' % options.firmware)

  fw_size = os.path.getsize(options.firmware)
  if fw_size > MAX_FW_SIZE:
    raise FlashPDError('Firmware too large %d/%d' % (fw_size, MAX_FW_SIZE))

  return options


def main_function():
  options = parse_args()

  loglevel = logging.INFO
  log_format = '%(asctime)s - %(name)s - %(levelname)s'
  if options.debug:
    loglevel = logging.DEBUG
    log_format += ' - %(filename)s:%(lineno)d:%(funcName)s'
  log_format += ' - %(message)s'
  logging.basicConfig(level=loglevel, format=log_format)

  flash_pd(options)


def main():
  """Main function wrapper to catch exceptions properly."""
  try:
    main_function()
  except KeyboardInterrupt:
    sys.exit(0)
  except FlashPDError as e:
    print 'Error: ', e.message
    sys.exit(1)

if __name__ == '__main__':
  main()
