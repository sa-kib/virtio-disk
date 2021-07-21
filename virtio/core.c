#include <linux/virtio_ring.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <sys/uio.h>
#include <stdlib.h>

#include "kvm/virtio.h"
#include "kvm/virtio-mmio.h"
#include "kvm/util.h"
#include "kvm/kvm.h"

#include "../demu.h"

const char* virtio_trans_name(enum virtio_trans trans)
{
	if (trans == VIRTIO_PCI)
		return "pci";
	else if (trans == VIRTIO_MMIO)
		return "mmio";
	return "unknown";
}

void virt_queue__used_idx_advance(struct virt_queue *queue, u16 jump)
{
	u16 idx = virtio_guest_to_host_u16(queue, queue->vring.used->idx);

	/*
	 * Use wmb to assure that used elem was updated with head and len.
	 * We need a wmb here since we can't advance idx unless we're ready
	 * to pass the used element to the guest.
	 */
	xen_wmb();
	idx += jump;
	queue->vring.used->idx = virtio_host_to_guest_u16(queue, idx);

	/*
	 * Use wmb to assure used idx has been increased before we signal the guest.
	 * Without a wmb here the guest may ignore the queue since it won't see
	 * an updated idx.
	 */
	xen_wmb();
}

struct vring_used_elem *
virt_queue__set_used_elem_no_update(struct virt_queue *queue, u32 head,
				    u32 len, u16 offset)
{
	struct vring_used_elem *used_elem;
	u16 idx = virtio_guest_to_host_u16(queue, queue->vring.used->idx);

	idx += offset;
	used_elem	= &queue->vring.used->ring[idx % queue->vring.num];
	used_elem->id	= virtio_host_to_guest_u32(queue, head);
	used_elem->len	= virtio_host_to_guest_u32(queue, len);

	return used_elem;
}

struct vring_used_elem *virt_queue__set_used_elem(struct virt_queue *queue, u32 head, u32 len)
{
	struct vring_used_elem *used_elem;

	used_elem = virt_queue__set_used_elem_no_update(queue, head, len, 0);
	virt_queue__used_idx_advance(queue, 1);

	return used_elem;
}

static inline bool virt_desc__test_flag(struct virt_queue *vq,
					struct vring_desc *desc, u16 flag)
{
	return !!(virtio_guest_to_host_u16(vq, desc->flags) & flag);
}

/*
 * Each buffer in the virtqueues is actually a chain of descriptors.  This
 * function returns the next descriptor in the chain, or max if we're at the
 * end.
 */
static unsigned next_desc(struct virt_queue *vq, struct vring_desc *desc,
			  unsigned int i, unsigned int max)
{
	unsigned int next;

	/* If this descriptor says it doesn't chain, we're done. */
	if (!virt_desc__test_flag(vq, &desc[i], VRING_DESC_F_NEXT))
		return max;

	next = virtio_guest_to_host_u16(vq, desc[i].next);

	/* Ensure they're not leading us off end of descriptors. */
	return min(next, max);
}

u16 virt_queue__get_head_iov(struct virt_queue *vq, struct iovec iov[], u16 *out, u16 *in, u16 head, struct kvm *kvm)
{
	struct vring_desc *desc;
	u16 idx;
	u16 max;
#ifndef MAP_IN_ADVANCE
	u32 mapped = 0;
#endif

	idx = head;
	*out = *in = 0;
	max = vq->vring.num;
	desc = vq->vring.desc;

	if (virt_desc__test_flag(vq, &desc[idx], VRING_DESC_F_INDIRECT)) {
		max = virtio_guest_to_host_u32(vq, desc[idx].len) / sizeof(struct vring_desc);
#ifndef MAP_IN_ADVANCE
		mapped = virtio_guest_to_host_u32(vq, desc[idx].len);
		desc = demu_map_guest_range(virtio_guest_to_host_u64(vq, desc[idx].addr),
				virtio_guest_to_host_u32(vq, desc[idx].len));
#else
		desc = demu_get_host_addr(virtio_guest_to_host_u64(vq, desc[idx].addr));
#endif
		idx = 0;
	}

	do {
		/* Grab the first descriptor, and check it's OK. */
		iov[*out + *in].iov_len = virtio_guest_to_host_u32(vq, desc[idx].len);
		/* Map all descriptors */
#ifndef MAP_IN_ADVANCE
		iov[*out + *in].iov_base = demu_map_guest_range(
				virtio_guest_to_host_u64(vq, desc[idx].addr),
				virtio_guest_to_host_u32(vq, desc[idx].len));
#else
		iov[*out + *in].iov_base = demu_get_host_addr(
				virtio_guest_to_host_u64(vq, desc[idx].addr));
#endif

		/* If this is an input descriptor, increment that count. */
		if (virt_desc__test_flag(vq, &desc[idx], VRING_DESC_F_WRITE))
			(*in)++;
		else
			(*out)++;
	} while ((idx = next_desc(vq, desc, idx, max)) != max);

#ifndef MAP_IN_ADVANCE
	if (mapped)
		demu_unmap_guest_range(desc, mapped);
#endif

	return head;
}

u16 virt_queue__get_iov(struct virt_queue *vq, struct iovec iov[], u16 *out, u16 *in, struct kvm *kvm)
{
	u16 head;

	head = virt_queue__pop(vq);

	return virt_queue__get_head_iov(vq, iov, out, in, head, kvm);
}

/* in and out are relative to guest */
u16 virt_queue__get_inout_iov(struct kvm *kvm, struct virt_queue *queue,
			      struct iovec in_iov[], struct iovec out_iov[],
			      u16 *in, u16 *out)
{
	struct vring_desc *desc;
	u16 head, idx;

	idx = head = virt_queue__pop(queue);
	*out = *in = 0;
	do {
		u64 addr;
		desc = virt_queue__get_desc(queue, idx);
		addr = virtio_guest_to_host_u64(queue, desc->addr);
		if (virt_desc__test_flag(queue, desc, VRING_DESC_F_WRITE)) {
#ifndef MAP_IN_ADVANCE
			in_iov[*in].iov_base = demu_map_guest_range(addr,
					virtio_guest_to_host_u32(queue, desc->len));
#else
			in_iov[*in].iov_base = demu_get_host_addr(addr);
#endif
			in_iov[*in].iov_len = virtio_guest_to_host_u32(queue, desc->len);
			(*in)++;
		} else {
#ifndef MAP_IN_ADVANCE
			out_iov[*out].iov_base = demu_map_guest_range(addr,
					virtio_guest_to_host_u32(queue, desc->len));
#else
			out_iov[*out].iov_base = demu_get_host_addr(addr);
#endif
			out_iov[*out].iov_len = virtio_guest_to_host_u32(queue, desc->len);
			(*out)++;
		}
		if (virt_desc__test_flag(queue, desc, VRING_DESC_F_NEXT))
			idx = virtio_guest_to_host_u16(queue, desc->next);
		else
			break;
	} while (1);

	return head;
}

void virtio_exit_vq(struct kvm *kvm, struct virtio_device *vdev,
			   void *dev, int num)
{
	struct virt_queue *vq = vdev->ops->get_vq(kvm, dev, num);

	if (vq->enabled && vdev->ops->exit_vq)
		vdev->ops->exit_vq(kvm, dev, num);
	memset(vq, 0, sizeof(*vq));
}

int virtio__get_dev_specific_field(int offset, bool msix, u32 *config_off)
{
	if (msix) {
		if (offset < 4)
			return VIRTIO_PCI_O_MSIX;
		else
			offset -= 4;
	}

	*config_off = offset;

	return VIRTIO_PCI_O_CONFIG;
}

bool virtio_queue__should_signal(struct virt_queue *vq)
{
	u16 old_idx, new_idx, event_idx;

	if (!vq->use_event_idx) {
		/*
		 * When VIRTIO_RING_F_EVENT_IDX isn't negotiated, interrupt the
		 * guest if it didn't explicitly request to be left alone.
		 */
		return !(virtio_guest_to_host_u16(vq, vq->vring.avail->flags) &
			 VRING_AVAIL_F_NO_INTERRUPT);
	}

	old_idx		= vq->last_used_signalled;
	new_idx		= virtio_guest_to_host_u16(vq, vq->vring.used->idx);
	event_idx	= virtio_guest_to_host_u16(vq, vring_used_event(&vq->vring));

	if (vring_need_event(event_idx, new_idx, old_idx)) {
		vq->last_used_signalled = new_idx;
		return true;
	}

	return false;
}

void virtio_set_guest_features(struct kvm *kvm, struct virtio_device *vdev,
			       void *dev, u32 features)
{
	/* TODO: fail negotiation if features & ~host_features */

	vdev->features = features;
	vdev->ops->set_guest_features(kvm, dev, features);
}

void virtio_notify_status(struct kvm *kvm, struct virtio_device *vdev,
			  void *dev, u8 status)
{
	u32 ext_status = status;

	vdev->status &= ~VIRTIO_CONFIG_S_MASK;
	vdev->status |= status;

	/* Add a few hints to help devices */
	if ((status & VIRTIO_CONFIG_S_DRIVER_OK) &&
	    !(vdev->status & VIRTIO__STATUS_START)) {
		vdev->status |= VIRTIO__STATUS_START;
		ext_status |= VIRTIO__STATUS_START;

	} else if (!status && (vdev->status & VIRTIO__STATUS_START)) {
		vdev->status &= ~VIRTIO__STATUS_START;
		ext_status |= VIRTIO__STATUS_STOP;

		/*
		 * Reset virtqueues and stop all traffic now, so that the device
		 * can safely reset the backend in notify_status().
		 */
		if (ext_status & VIRTIO__STATUS_STOP)
			vdev->ops->reset(kvm, vdev);
	}

	if (vdev->ops->notify_status)
		vdev->ops->notify_status(kvm, dev, ext_status);
}

int virtio_init(struct kvm *kvm, void *dev, struct virtio_device *vdev,
		struct virtio_ops *ops, enum virtio_trans trans,
		int device_id, int subsys_id, int class, u32 addr, u8 irq)
{
	void *virtio;
	int r;

	switch (trans) {
	case VIRTIO_MMIO:
		virtio = calloc(sizeof(struct virtio_mmio), 1);
		if (!virtio)
			return -ENOMEM;
		vdev->virtio			= virtio;
		vdev->ops			= ops;
		vdev->ops->signal_vq		= virtio_mmio_signal_vq;
		vdev->ops->signal_config	= virtio_mmio_signal_config;
		vdev->ops->init			= virtio_mmio_init;
		vdev->ops->exit			= virtio_mmio_exit;
		vdev->ops->reset		= virtio_mmio_reset;
		r = vdev->ops->init(kvm, dev, vdev, device_id, subsys_id, class, addr, irq);
		break;
	default:
		r = -1;
	};

	return r;
}

