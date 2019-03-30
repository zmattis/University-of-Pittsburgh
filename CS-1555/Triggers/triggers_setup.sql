-- setup tables

drop table if exists students cascade;

create table students (
  id integer primary key,
  name varchar(24)
);

insert into students values(1, 'Alice');
insert into students values(2, 'Bob');
insert into students values(3, 'Charlie');
insert into students values(4, 'Denise');
insert into students values(5, 'Edward');

drop table if exists courses cascade;

create table courses (
  num varchar(6) primary key,
  open boolean not null,
  enrolled integer default 0,
  lim integer default 3
);

insert into courses values('CS1555', True);
insert into courses values('CS1501', True);
insert into courses values('CS1520', True);

drop table if exists enrollment cascade;

create table enrollment (
  student integer references students(id),
  course varchar(6) references courses(num)
);

-- insert starting data

insert into enrollment values(1, 'CS1501');
insert into enrollment values(2, 'CS1501');
update courses set enrolled = 2 where num = 'CS1501';

insert into enrollment values(1, 'CS1555');
update courses set enrolled = 1 where num = 'CS1555';

insert into enrollment values(4, 'CS1520');
insert into enrollment values(5, 'CS1520');
update courses set enrolled = 2 where num = 'CS1520';