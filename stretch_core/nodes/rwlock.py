#! /usr/bin/env python3
"""rwlock.py
This write-preferring implementation of the readwrite lock
was made available via the Creative Commons license at:
https://stackoverflow.com/a/22109979
"""

import threading

class RWLock:
    """ Non-reentrant write-preferring rwlock. """
    DEBUG = 0

    def __init__(self):
        self.lock = threading.Lock()

        self.active_writer_lock = threading.Lock()
        # The total number of writers including the active writer and
        # those blocking on active_writer_lock or readers_finished_cond.
        self.writer_count = 0

        # Number of events that are blocking on writers_finished_cond.
        self.waiting_reader_count = 0

        # Number of events currently using the resource.
        self.active_reader_count = 0

        self.readers_finished_cond = threading.Condition(self.lock)
        self.writers_finished_cond = threading.Condition(self.lock)

        class _ReadAccess:
            def __init__(self, rwlock):
                self.rwlock = rwlock
            def __enter__(self):
                self.rwlock.acquire_read()
                return self.rwlock
            def __exit__(self, type, value, tb):
                self.rwlock.release_read()
        # support for the with statement
        self.read_access = _ReadAccess(self)

        class _WriteAccess:
            def __init__(self, rwlock):
                self.rwlock = rwlock
            def __enter__(self):
                self.rwlock.acquire_write()
                return self.rwlock
            def __exit__(self, type, value, tb):
                self.rwlock.release_write()
        # support for the with statement
        self.write_access = _WriteAccess(self)

        if self.DEBUG:
            self.active_readers = set()
            self.active_writer = None

    def acquire_read(self):
        with self.lock:
            if self.DEBUG:
                me = threading.currentThread()
                assert me not in self.active_readers, 'This thread has already acquired read access and this lock isn\'t reader-reentrant!'
                assert me != self.active_writer, 'This thread already has write access, release that before acquiring read access!'
                self.active_readers.add(me)
            if self.writer_count:
                self.waiting_reader_count += 1
                self.writers_finished_cond.wait()
                # Even if the last writer thread notifies us it can happen that a new
                # incoming writer thread acquires the lock earlier than this reader
                # thread so we test for the writer_count after each wait()...
                # We also protect ourselves from spurious wakeups that happen with some POSIX libraries.
                while self.writer_count:
                    self.writers_finished_cond.wait()
                self.waiting_reader_count -= 1
            self.active_reader_count += 1

    def release_read(self):
        with self.lock:
            if self.DEBUG:
                me = threading.currentThread()
                assert me in self.active_readers, 'Trying to release read access when it hasn\'t been acquired by this thread!'
                self.active_readers.remove(me)
            assert self.active_reader_count > 0
            self.active_reader_count -= 1
            if not self.active_reader_count and self.writer_count:
                self.readers_finished_cond.notifyAll()

    def acquire_write(self):
        with self.lock:
            if self.DEBUG:
                me = threading.currentThread()
                assert me not in self.active_readers, 'This thread already has read access - release that before acquiring write access!'
                assert me != self.active_writer, 'This thread already has write access and this lock isn\'t writer-reentrant!'
            self.writer_count += 1
            if self.active_reader_count:
                self.readers_finished_cond.wait()
                while self.active_reader_count:
                    self.readers_finished_cond.wait()

        self.active_writer_lock.acquire()
        if self.DEBUG:
            self.active_writer = me

    def release_write(self):
        if not self.DEBUG:
            self.active_writer_lock.release()
        with self.lock:
            if self.DEBUG:
                me = threading.currentThread()
                assert me == self.active_writer, 'Trying to release write access when it hasn\'t been acquired by this thread!'
                self.active_writer = None
                self.active_writer_lock.release()
            assert self.writer_count > 0
            self.writer_count -= 1
            if not self.writer_count and self.waiting_reader_count:
                self.writers_finished_cond.notifyAll()

    def get_state(self):
        with self.lock:
            return (self.writer_count, self.waiting_reader_count, self.active_reader_count)
