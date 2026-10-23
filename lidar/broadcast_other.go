//go:build !unix

/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 *
 * Contributors:
 *   Gabriel Axheim Gustafsson, Luleå - initial implementation
 ***************************************************************************SDG*/

package main

import "syscall"

// allowBroadcast does nothing here. Off Unix the discovery broadcast may be
// refused, and deviceIP has to be set.
func allowBroadcast(network, address string, c syscall.RawConn) error { return nil }
