#!/bin/bash
# set_irq_affinity.sh — bind non-RT interrupts to little cores (CPU 0-3)
#
# RK3588 CPU topology: CPUs 0-3 are Cortex-A55 (little), CPUs 4-7 are A76 (big).
# Big cores are isolated (isolcpus=4-7) for RT tasks (CAN gateway, EKF).
# This script ensures kernel-side ISRs don't compete with RT threads on big cores.

LITTLE_MASK="0f"  # CPU 0-3 bitmask

# CAN0 interrupt — IRQ number varies, find it dynamically
CAN_IRQ=$(grep -w can0 /proc/interrupts 2>/dev/null | awk -F: '{print $1}' | tr -d ' ')
if [ -n "$CAN_IRQ" ]; then
	echo "$LITTLE_MASK" > "/proc/irq/$CAN_IRQ/smp_affinity"
fi

# USB controllers (ehci/ohci/xhci)
for irq in $(grep -E 'ehci|ohci|xhci' /proc/interrupts 2>/dev/null | awk -F: '{print $1}'); do
	[ -n "$irq" ] && echo "$LITTLE_MASK" > "/proc/irq/$irq/smp_affinity"
done

# MMC/SD (eMMC, SD card)
for irq in $(grep -w mmc0 /proc/interrupts 2>/dev/null | awk -F: '{print $1}'); do
	[ -n "$irq" ] && echo "$LITTLE_MASK" > "/proc/irq/$irq/smp_affinity"
done

# Ethernet
for irq in $(grep -w eth0 /proc/interrupts 2>/dev/null | awk -F: '{print $1}'); do
	[ -n "$irq" ] && echo "$LITTLE_MASK" > "/proc/irq/$irq/smp_affinity"
done

# SPI
for irq in $(grep -E '[0-9a-f]+\.spi' /proc/interrupts 2>/dev/null | awk -F: '{print $1}'); do
	[ -n "$irq" ] && echo "$LITTLE_MASK" > "/proc/irq/$irq/smp_affinity"
done

echo "IRQ affinity set: non-RT interrupts → CPU 0-3"
