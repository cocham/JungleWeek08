#include "userprog/syscall.h"
#include <stdio.h>
#include <string.h>
#include <syscall-nr.h>
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "threads/loader.h"
#include "userprog/gdt.h"
#include "threads/flags.h"
#include "intrinsic.h"
#include "threads/init.h"
#include "threads/synch.h"
#include "threads/vaddr.h"
#include "threads/mmu.h"
#include "threads/palloc.h"
#include "userprog/process.h"
#include "filesys/filesys.h"
#include "filesys/file.h"
#include "devices/input.h"

void syscall_entry (void);
void syscall_handler (struct intr_frame *);

/* 파일 시스템 전체를 한 번에 한 프로세스만 쓰게 막는 락.
   유저 포인터 검사는 이 락을 잡기 전에 끝낸다. 락을 쥔 채 exit 하면 락이 풀리지 않는다. */
struct lock filesys_lock;

static void sys_halt (void);
static tid_t sys_fork (const char *thread_name, struct intr_frame *f);
static int sys_exec (const char *cmd_line);
static int sys_wait (tid_t pid);
static bool sys_create (const char *file, unsigned initial_size);
static bool sys_remove (const char *file);
static int sys_open (const char *file);
static int sys_filesize (int fd);
static int sys_read (int fd, void *buffer, unsigned size);
static int sys_write (int fd, const void *buffer, unsigned size);
static void sys_seek (int fd, unsigned position);
static unsigned sys_tell (int fd);
static void sys_close (int fd);

/* System call.
 *
 * Previously system call services was handled by the interrupt handler
 * (e.g. int 0x80 in linux). However, in x86-64, the manufacturer supplies
 * efficient path for requesting the system call, the `syscall` instruction.
 *
 * The syscall instruction works by reading the values from the the Model
 * Specific Register (MSR). For the details, see the manual. */

#define MSR_STAR 0xc0000081         /* Segment selector msr */
#define MSR_LSTAR 0xc0000082        /* Long mode SYSCALL target */
#define MSR_SYSCALL_MASK 0xc0000084 /* Mask for the eflags */

void
syscall_init (void) {
	write_msr(MSR_STAR, ((uint64_t)SEL_UCSEG - 0x10) << 48  |
			((uint64_t)SEL_KCSEG) << 32);
	write_msr(MSR_LSTAR, (uint64_t) syscall_entry);

	/* The interrupt service rountine should not serve any interrupts
	 * until the syscall_entry swaps the userland stack to the kernel
	 * mode stack. Therefore, we masked the FLAG_FL. */
	write_msr(MSR_SYSCALL_MASK,
			FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);

	/* 첫 유저 프로세스가 파일을 열기 전에 준비돼 있어야 한다. */
	lock_init (&filesys_lock);
}

/* The main system call interface */
void
syscall_handler (struct intr_frame *f) {
	/* 시스템 콜 번호는 rax, 인자는 rdi · rsi · rdx · r10 · r8 · r9 순서로 들어온다.
	   반환값이 있는 시스템 콜은 f->R.rax 에 써 넣으면 유저 쪽 함수의 반환값이 된다. */
	switch (f->R.rax) {
		case SYS_HALT:
			sys_halt ();
			break;
		case SYS_EXIT:
			sys_exit ((int) f->R.rdi);
			break;
		case SYS_FORK:
			f->R.rax = sys_fork ((const char *) f->R.rdi, f);
			break;
		case SYS_EXEC:
			f->R.rax = sys_exec ((const char *) f->R.rdi);
			break;
		case SYS_WAIT:
			f->R.rax = sys_wait ((tid_t) f->R.rdi);
			break;
		case SYS_CREATE:
			f->R.rax = sys_create ((const char *) f->R.rdi, (unsigned) f->R.rsi);
			break;
		case SYS_REMOVE:
			f->R.rax = sys_remove ((const char *) f->R.rdi);
			break;
		case SYS_OPEN:
			f->R.rax = sys_open ((const char *) f->R.rdi);
			break;
		case SYS_FILESIZE:
			f->R.rax = sys_filesize ((int) f->R.rdi);
			break;
		case SYS_READ:
			f->R.rax = sys_read ((int) f->R.rdi, (void *) f->R.rsi, (unsigned) f->R.rdx);
			break;
		case SYS_WRITE:
			f->R.rax = sys_write ((int) f->R.rdi, (const void *) f->R.rsi, (unsigned) f->R.rdx);
			break;
		case SYS_SEEK:
			sys_seek ((int) f->R.rdi, (unsigned) f->R.rsi);
			break;
		case SYS_TELL:
			f->R.rax = sys_tell ((int) f->R.rdi);
			break;
		case SYS_CLOSE:
			sys_close ((int) f->R.rdi);
			break;
		default:
			/* 모르는 번호면 유저 프로그램이 잘못 부른 것이므로 프로세스를 끝낸다. */
			sys_exit (-1);
	}
}

/* 핀토스를 끈다. (A) */
static void
sys_halt (void) {
	/* 프로세스가 끝나는 것이 아니므로 종료 메시지는 찍지 않는다. */
	power_off ();
}

/* 종료 상태를 남기고 프로세스를 끝낸다.
   종료 메시지는 process_exit () 에서 출력한다. (A) */
void
sys_exit (int status) {
	thread_current ()->exit_status = status;
	thread_exit ();
}

/* 현재 프로세스를 복제한다. 부모의 유저 레지스터가 담긴 f 를 자식에게 넘겨야 한다. (C) */
static tid_t
sys_fork (const char *thread_name UNUSED, struct intr_frame *f UNUSED) {
	/* TODO (C): 구현. */
	sys_exit (-1);
}

/* 현재 프로세스를 다른 프로그램으로 바꾼다. 성공하면 돌아오지 않는다. (A) */
static int
sys_exec (const char *cmd_line) {
	char *cmd_copy;

	/* 커널이 문자열을 읽기 전에 검사한다. 잘못된 주소면 이 프로세스만 끝난다. */
	validate_user_string (cmd_line);

	/* process_exec () 는 load 전에 지금 유저 메모리를 지우고, 끝나면 받은 문자열을
	   페이지로 해제한다. 그래서 유저 문자열을 그대로 넘기지 않고 커널 페이지에 복사한다. */
	cmd_copy = palloc_get_page (0);
	if (cmd_copy == NULL)
		sys_exit (-1);
	strlcpy (cmd_copy, cmd_line, PGSIZE);

	/* 돌아왔다면 로드에 실패한 것이다. cmd_copy 는 process_exec () 안에서 이미 해제됐다. */
	if (process_exec (cmd_copy) < 0)
		sys_exit (-1);
	return -1;
}

/* 자식이 끝날 때까지 기다리고 종료 상태를 받는다. (C) */
static int
sys_wait (tid_t pid) {
	return process_wait (pid);
}

/* 파일을 만든다. 이미 있거나 만들 수 없으면 false. (B) */
static bool
sys_create (const char *file, unsigned initial_size) {
	bool success;

	validate_user_string (file);

	lock_acquire (&filesys_lock);
	success = filesys_create (file, initial_size);
	lock_release (&filesys_lock);
	return success;
}

/* 파일을 지운다. 없거나 지울 수 없으면 false. (B) */
static bool
sys_remove (const char *file) {
	bool success;

	validate_user_string (file);

	lock_acquire (&filesys_lock);
	success = filesys_remove (file);
	lock_release (&filesys_lock);
	return success;
}

/* 파일을 열고 fd 를 돌려준다. 열 수 없거나 fd 자리가 없으면 -1. (B) */
static int
sys_open (const char *file) {
	struct file *opened_file;
	int fd;

	validate_user_string (file);

	lock_acquire (&filesys_lock);
	opened_file = filesys_open (file);
	if (opened_file == NULL) {
		lock_release (&filesys_lock);
		return -1;
	}

	/* fd 테이블이 꽉 찼으면 방금 연 파일을 닫아야 파일 구조체가 새지 않는다. */
	fd = fd_alloc (opened_file);
	if (fd == -1) {
		file_close (opened_file);
		lock_release (&filesys_lock);
		return -1;
	}

	lock_release (&filesys_lock);
	return fd;
}

/* 파일 크기를 바이트 단위로 돌려준다. 열린 fd 가 아니면 -1. (B) */
static int
sys_filesize (int fd) {
	struct file *file = fd_lookup (fd);
	int size;

	if (file == NULL)
		return -1;

	lock_acquire (&filesys_lock);
	size = file_length (file);
	lock_release (&filesys_lock);
	return size;
}

/* 파일 또는 키보드(fd 0)에서 읽고 읽은 바이트 수를 돌려준다. 열린 fd 가 아니면 -1. (B) */
static int
sys_read (int fd, void *buffer, unsigned size) {
	struct file *file;
	int bytes;

	validate_user_buffer (buffer, size);

	/* fd 0 은 키보드. 파일 시스템을 쓰지 않으므로 락이 필요 없다. */
	if (fd == 0) {
		uint8_t *p = buffer;

		for (unsigned i = 0; i < size; i++)
			p[i] = input_getc ();
		return size;
	}

	file = fd_lookup (fd);
	if (file == NULL)
		return -1;

	lock_acquire (&filesys_lock);
	bytes = file_read (file, buffer, size);
	lock_release (&filesys_lock);
	return bytes;
}

/* 콘솔(fd 1, A) 또는 파일(B)에 쓰고 쓴 바이트 수를 돌려준다. 열린 fd 가 아니면 -1. */
static int
sys_write (int fd, const void *buffer, unsigned size) {
	struct file *file;
	int bytes;

	validate_user_buffer (buffer, size);

	/* fd 1 은 콘솔. 여러 번 나눠 출력하면 다른 프로세스 출력과 섞일 수 있어
	   콘솔 락을 한 번만 잡는 putbuf () 로 한 번에 출력한다. */
	if (fd == 1) {
		putbuf (buffer, size);
		return size;
	}

	file = fd_lookup (fd);
	if (file == NULL)
		return -1;

	lock_acquire (&filesys_lock);
	bytes = file_write (file, buffer, size);
	lock_release (&filesys_lock);
	return bytes;
}

/* 다음에 읽거나 쓸 위치를 position 으로 옮긴다. (B) */
static void
sys_seek (int fd, unsigned position) {
	struct file *file = fd_lookup (fd);

	/* file_seek () 는 off_t (int32) 를 받으므로 그보다 큰 위치는 무시한다. */
	if (file == NULL || position > INT32_MAX)
		return;

	lock_acquire (&filesys_lock);
	file_seek (file, position);
	lock_release (&filesys_lock);
}

/* 다음에 읽거나 쓸 위치를 돌려준다. 열린 fd 가 아니면 -1. (B) */
static unsigned
sys_tell (int fd) {
	struct file *file = fd_lookup (fd);
	int pos;

	if (file == NULL)
		return -1;

	lock_acquire (&filesys_lock);
	pos = file_tell (file);
	lock_release (&filesys_lock);
	return pos;
}

/* fd 를 닫는다. 열린 fd 가 아니면 아무것도 하지 않는다. (B) */
static void
sys_close (int fd) {
	struct file *file = fd_lookup (fd);

	if (file == NULL)
		return;

	/* 닫기 전에 칸부터 비워, 같은 fd 를 다시 close 해도 무시되게 한다. */
	thread_current ()->fd_table[fd] = NULL;

	lock_acquire (&filesys_lock);
	file_close (file);
	lock_release (&filesys_lock);
}

/* 유저가 넘긴 주소 하나를 검사한다. NULL, 커널 영역, 매핑 안 된 주소면 exit (-1). (A) */
void
validate_user_addr (const void *uaddr) {
	/* pml4_get_page () 는 커널 주소를 받으면 ASSERT 로 멈추므로 커널 영역부터 거른다. */
	if (uaddr == NULL || !is_user_vaddr (uaddr)
			|| pml4_get_page (thread_current ()->pml4, uaddr) == NULL)
		sys_exit (-1);
}

/* 버퍼 전체(buffer ~ buffer + size - 1)를 검사한다. 페이지 경계 포함. (A) */
void
validate_user_buffer (const void *buffer, unsigned size) {
	const uint8_t *p = buffer;

	for (unsigned i = 0; i < size; i++)
		validate_user_addr (p + i);
}

/* 문자열을 '\0' 까지 검사한다. (A) */
void
validate_user_string (const char *str) {
	const char *p = str;

	/* 길이를 모르니 한 글자씩 가되, 커널이 읽기 전에 그 칸부터 검사한다. */
	for (;;) {
		validate_user_addr (p);
		if (*p == '\0')
			return;
		p++;
	}
}

/* 빈 fd 칸에 파일을 넣고 그 번호를 돌려준다. 자리가 없으면 -1. (B) */
int
fd_alloc (struct file *file) {
	struct thread *cur = thread_current ();
	int fd;
	int checked = 0;

	if (file == NULL || cur->fd_table == NULL)
		return -1;

	/* 0, 1 은 콘솔용이므로 2 부터 쓴다. 매번 처음부터 찾지 않도록
	   지난번에 준 번호 다음(next_fd)부터 찾는다. */
	fd = cur->next_fd;
	if (fd < 2 || fd >= FD_MAX)
		fd = 2;

	/* 2 ~ FD_MAX - 1 을 한 바퀴 돌며 빈 칸을 찾는다. 끝에 닿으면 2 로 돌아간다. */
	while (checked < FD_MAX - 2) {
		if (cur->fd_table[fd] == NULL) {
			cur->fd_table[fd] = file;
			cur->next_fd = fd + 1 < FD_MAX ? fd + 1 : 2;
			return fd;
		}

		fd++;
		if (fd >= FD_MAX)
			fd = 2;
		checked++;
	}
	return -1;
}

/* fd 로 열린 파일을 찾는다. 없거나 범위 밖이면 NULL. (B) */
struct file *
fd_lookup (int fd) {
	struct thread *cur = thread_current ();

	/* 0, 1 은 콘솔이라 파일이 아니다. */
	if (fd < 2 || fd >= FD_MAX || cur->fd_table == NULL)
		return NULL;
	return cur->fd_table[fd];
}