/**
 * @author Zachary M. Mattis
 * CS 1555
 * Triggers
 * March 29, 2019
 *
 * This SQL file provides the SQL
 * implementations for the database
 * triggers.
 */

-- Enrollment Function
CREATE OR REPLACE FUNCTION tr_enroll() RETURNS TRIGGER AS $tr_enroll$
  DECLARE
    update_count integer;

  BEGIN

    SELECT COUNT(*) INTO update_count FROM enrollment WHERE course = NEW.course;

    UPDATE courses
    SET
      enrolled = update_count,
      open = CASE WHEN update_count = ( SELECT lim FROM courses WHERE num = NEW.course )
        THEN False ELSE True END
      WHERE num = NEW.course;

    RETURN NEW;
  END;
$tr_enroll$ LANGUAGE plpgsql;

-- Enrollment Trigger
CREATE TRIGGER TR_ENROLL
AFTER INSERT
ON enrollment
FOR EACH ROW EXECUTE PROCEDURE tr_enroll();


-- Restrict Function
CREATE OR REPLACE FUNCTION tr_restrict() RETURNS TRIGGER AS $tr_restrict$
  DECLARE
    update_count integer;

  BEGIN

    SELECT COUNT(*) INTO update_count FROM enrollment WHERE course = NEW.course;

    -- Class limits
    IF update_count >= ( SELECT lim FROM courses WHERE num = NEW.course) THEN
      RAISE EXCEPTION '% is already full...', NEW.course;
    END IF;

    RETURN NEW;
  END;
$tr_restrict$ LANGUAGE plpgsql;

-- Restrict Trigger
CREATE TRIGGER TR_RESTRICT
BEFORE INSERT
ON enrollment
FOR EACH ROW EXECUTE PROCEDURE tr_restrict();

-- Test INSERT
--insert into enrollment values(3, 'CS1555');
--insert into enrollment values(4, 'CS1555');
--insert into enrollment values(5, 'CS1555');
